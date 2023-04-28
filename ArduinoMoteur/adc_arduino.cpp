#include <Arduino.h>
#include "encoder_arduino.h"
//#define USE_ADC_DOUBLE_BUFFERING
//#define TEST_WITH_VISUAL_STUDIO_CODE

#define DIRECT_PIN_READ(base, mask) (((*(base)) & (mask)) ? 1 : 0)
#define USE_MEDIAN
#ifdef USE_MEDIAN
#define N_MEDIAN 5
#endif
#define MAX_NB_ADC 16
#define DMA_STATE_EVEN 1 /* indicate that dma use even buffer */
#define DMA_STATE_ODD 2  /* indicate that dma use odd buffer */
typedef struct
{
    long adc_count;
    int nb_channel;
#ifdef USE_MEDIAN
    uint16_t u_median[MAX_NB_ADC][N_MEDIAN];
    int8_t k_median[MAX_NB_ADC][N_MEDIAN];
#endif
    uint16_t adc_buffer_even[MAX_NB_ADC]; // holds the raw data from the analog to digital
    int32_t flt_output[MAX_NB_ADC];       // filtered ouput 1/(1-[1-a].z^-1) , a = 2^-shift
    uint8_t shift;                        // for output filter : (1-a)/(1-[1-a].z^-1) , a = 2^-shift
#ifdef USE_ADC_DOUBLE_BUFFERING
    uint16_t adc_buffer_odd[MAX_NB_ADC]; // holds the raw data from the analog to digital
    uint8_t dma_state;
#endif
} struct_adcs;
volatile struct_adcs adcs;
void setup_AtoD(volatile struct_adcs &adcs, uint8_t nb_chann, uint8_T channels[], uint8_t gain_124[], uint8_t offsets_01[], uint8_t shift)
{

    adcs.nb_channel = nb_chann;
    adcs.shift = shift;
    adcs.adc_count = 0;
    for (int i = 0; i < nb_chann; i++)
    {
        adcs.adc_buffer_even[i] = 0;
#ifdef USE_MEDIAN
        for (int n = 0; n < N_MEDIAN; n++)
        {
            adcs.u_median[i][n] = 0;
            adcs.k_median[i][n] = n;
        }
#endif
#ifdef USE_ADC_DOUBLE_BUFFERING
        adcs.adc_buffer_odd[i] = 0;
#endif
        if (adcs.shift > 0)
        {
            adcs.flt_output[i] = 0;
        }
    }
    pmc_enable_periph_clk(ID_ADC);       //power management controller told to turn on adc
    ADC->ADC_CR |= 1;                    //reset the adc
    ADC->ADC_MR = 0x9038ff00;            //this setting is used by arduino. ?
    ADC->ADC_MR &= 0xFFFFFFEF;           // bit 4 of ADC_MR =0 to get 12 bits resolution adc
    ADC->ADC_MR &= 0xFFFF00FF;           //mode register "prescale" zeroed out.
    ADC->ADC_MR |= ADC_MR_PRESCAL(0xFF); // ADC CLOCK=84MHZ/((255+1)*2)=164KHZ
    ADC->ADC_EMR |= (1 << 24);           // turn on channel numbers
    // configure sequential mode
    ADC->ADC_CHDR = 0xFFFFFFFF; // disable all channels
    uint32_T mask_CHER = 0, mask_SEQR1 = 0, mask_SEQR2 = 0, mask_CGR = 0, mask_COR = 0;
    for (int i = 0; i < nb_chann; i++)
    {
        mask_CHER |= (1 << i);
        uint32_T arduinoPin;
        uint32_T ulChannel;
        if (channels[i] != 15)
        {
            arduinoPin = channels[i] + A0;
            ulChannel = g_APinDescription[arduinoPin].ulADCChannelNumber;
        }
        else
        {
            // temperature sensor on pin 15 , see table 46.37 , V= 0.8V +/-15% at 27 °C, pente =2.65mv/°C +/-5%
            ulChannel = channels[i];
            ADC->ADC_ACR |= (1 << 4); // set TSON bot of register ADC_ACR, see 44.7.19 ADC Analog Control Register
        }
        if (i < 8)
        { // channels 0 to 7, see table 44.7.16 of arm-cortex-m3-sam3x8e
            mask_SEQR1 |= (ulChannel << (i * 4));
        }
        else
        {
            mask_SEQR2 |= (ulChannel << ((i - 8) * 4));
        }
        // gain scaling before conversion : see see table 44.7.16 of arm-cortex-m3-sam3x8e
        switch (gain_124[i])
        {
        case 1:
            mask_CGR |= (1 << (ulChannel * 2)); // gain works for real adc, not mux channel
                                                // mask_CGR |= (1 << ( i  * 2));
            break;
        case 2:
            mask_CGR |= (2 << (ulChannel * 2)); // gain works for real adc, not mux channel
            //  mask_CGR |= (2 << ( i  * 2));
            break;
        case 4:
            mask_CGR |= (3 << (ulChannel * 2)); // gain works for real adc, not mux channel
            //  mask_CGR |= (3 << ( i  * 2));
            break;
        }
        // offset Vref/2 before conversion : see see table 44.7.17 of arm-cortex-m3-sam3x8e
        if (offsets_01[i] == 1)
        { // DO NOT WORK, TODO
            //    mask_COR |= (1 << ulChannel); // apply offset
            //    mask_COR |= (1 << i); // apply offset
        }
    }                            // for i...
    ADC->ADC_MR |= 0x80000000;   //USEQ bit set, saying use the sequence
    ADC->ADC_SEQR1 = mask_SEQR1; //   adcs for mux channels 0 to 7
    ADC->ADC_SEQR2 = mask_SEQR2; //   adcs for mux channels 8 to 15
    ADC->ADC_CHER = mask_CHER;
    ADC->ADC_MR |= ADC_MR_ANACH; // allow individual settings for each channel
                                 // ADC->ADC_CGR = 0x30303030;//mask_CGR;
    ADC->ADC_CGR = mask_CGR;
    ADC->ADC_COR = mask_COR;

    // setup irq
    NVIC_EnableIRQ(ADC_IRQn);    // interrupt controller set to enable adc.
    ADC->ADC_IDR = ~((1 << 27)); // interrupt disable register, disables all interrupts but ENDRX
    ADC->ADC_IER = (1 << 27);    // interrupt enable register, enables only ENDRX

// following are the DMA controller registers for this peripheral
// "receive buffer address"
#ifdef USE_ADC_DOUBLE_BUFFERING
    adcs.dma_state = DMA_STATE_EVEN;               // indicate that dma use even buffer
#endif                                             // ADC initialisation
    ADC->ADC_RPR = (uint32_t)adcs.adc_buffer_even; // DMA receive pointer register  points to beginning of initial data to fill
    ADC->ADC_RCR = adcs.nb_channel;                //  receive counter set to nb channels
#ifdef USE_ADC_DOUBLE_BUFFERING

    // "next-buffer address"
    ADC->ADC_RNPR = (uint32_t)adcs.adc_buffer_odd; // next receive pointer register DMA global_ADCounts_Arrayfer  points to second set of data
#else
    ADC->ADC_RNPR = (uint32_t)adcs.adc_buffer_even;     // DMA receive pointer register  points to beginning of initial data to fill
#endif
    // and "next count"
    ADC->ADC_RNCR = adcs.nb_channel; //  and next counter is set to nb channels

    // "transmit control register"
    ADC->ADC_PTCR = 1; // transfer control register for the DMA is set to enable receiver channel requests
    // now that all things are set up, it is safe to start the ADC.....
    ADC->ADC_MR |= 0x80; // mode register of adc bit seven, free run, set to free running. starts ADC
}


#ifdef USE_MEDIAN
void tri_bulle(volatile uint16_t u[], volatile int8_t k[], int N) {
    int n_iter = 0;
    bool permut = true;
    int i;
    while (permut) {
        permut = false;
        n_iter++;
        for (i = 0; i < N - n_iter; i++) {
            if (u[i] > u[i + 1]) {
                permut = true;
                // on echange les deux elements
                int temp = u[i];
                u[i] = u[i + 1];
                u[i + 1] = temp;                
                temp = k[i];
                k[i] = k[i + 1];
                k[i + 1] = temp;
            }
        }
    }
}

void median(volatile uint16_t u[], volatile int8_t k[], int16_t unew, int N) {
    for (int i = 0; i < N; i++) {
        // recherche indice i de l'element le plus ancien
        k[i]++;
        if (k[i] == N) {
            // placement du nouveau 
            u[i] = unew ;
            k[i] = 0;
        }
    }
    tri_bulle(u, k, N);
}

void adc_update_median_ouput(volatile struct_adcs &adcs)
{

    for (int i = 0; i < adcs.nb_channel; i++)
    {
       median(adcs.u_median[i],adcs.k_median[i],adcs.adc_buffer_even[i]&0xfff,N_MEDIAN);
    }
}

#endif
void adc_update_filtered_ouput(volatile struct_adcs &adcs)
{
    if (adcs.shift <= 0)
        return;
    for (int i = 0; i < adcs.nb_channel; i++)
    {
        adcs.flt_output[i] -= adcs.flt_output[i] >> adcs.shift;
        #ifdef USE_MEDIAN
          adcs.flt_output[i]+=adcs.u_median[i][N_MEDIAN/2];
        #else
          adcs.flt_output[i] += (adcs.adc_buffer_even[i] & 0xfff);
        #endif
    }
}
void ADC_Handler()
{

    // for the ATOD: re-initialize DMA pointers and count
    int f = ADC->ADC_ISR; //   read the interrupt status register
    //adcs.last_value = adc_get_latest_value(ADC);
    if (f & (1 << 27))
    { /// check the bit "endrx"  in the status register
        adcs.adc_count++;
        #ifdef USE_MEDIAN
          adc_update_median_ouput(adcs);
        #endif
        adc_update_filtered_ouput(adcs);
#ifdef USE_ADC_DOUBLE_BUFFERING
        switch (adcs.dma_state)
        {
        case DMA_STATE_EVEN:
            /// set up the "next pointer register"
            ADC->ADC_RNPR = (uint32_t)adcs.adc_buffer_odd; // "receive next pointer" register set to adc_buffer_even

            adcs.dma_state = DMA_STATE_ODD;
            break;
        case DMA_STATE_ODD:
            /// set up the "next pointer register"
            ADC->ADC_RNPR = (uint32_t)adcs.adc_buffer_even; // "receive next pointer" register set to adc_buffer_even
            adcs.dma_state = DMA_STATE_EVEN;
            break;
        }
#else
        ADC->ADC_RNPR = (uint32_t)adcs.adc_buffer_even; // "receive next pointer" register set to adc_buffer_even
#endif
        // set up the "next count"
        ADC->ADC_RNCR = adcs.nb_channel; // "receive next" counter set to nb_channel
    }
}
int16_T getAdcValue(volatile struct_adcs &adcs, int i)
{
    if (adcs.shift > 0)
    { // use output filter
        return (adcs.flt_output[i] >> adcs.shift);
    }
#ifdef USE_MEDIAN
    return adcs.u_median[i][N_MEDIAN/2];             // "receive next pointer" register set to adc_buffer_even
#endif

#ifdef USE_ADC_DOUBLE_BUFFERING
    switch (adcs.dma_state)
    {
    case DMA_STATE_EVEN:
        return adcs.adc_buffer_odd[i] & 0xfff; // "receive next pointer" register set to adc_buffer_even

    case DMA_STATE_ODD:
        return adcs.adc_buffer_even[i] & 0xfff; // "receive next pointer" register set to adc_buffer_even
    }
    return 0;
#else
    return adcs.adc_buffer_even[i] & 0xfff;             // "receive next pointer" register set to adc_buffer_even
#endif

}
// Create an Encoder object and save it to the pointer
extern "C" void adcSetup(uint8_T nb_channels, uint8_T channels[], uint8_T gains_124[], uint8_T shift)
{
    uint8_T offsets_01[MAX_NB_ADC] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    setup_AtoD(adcs, nb_channels, channels, gains_124, offsets_01, shift);
}

// Read the position relative to the old position
// and reset the position to zero
extern "C" uint16_T readAdc(uint8_T num_channel)
{
    return getAdcValue(adcs, num_channel);
}
#ifdef TEST_WITH_VISUAL_STUDIO_CODE

void setup()
{
    uint8_T channels[3] = {0, 1, 15};
    uint8_T gains_124[3] = {1, 2, 1};  // gains wporks only for different ADC channels
    uint8_T offsets_01[3] = {0, 1, 0}; // offsets DO NOT WORK?...
    setup_AtoD(adcs, 3, channels, gains_124, offsets_01, 0);
    Serial.begin(115200);
}

void loop()
{
    while (1)
    {
        Serial.print(adcs.adc_count);
        Serial.print(":");

        for (int i = 0; i < adcs.nb_channel; i++)
        {
            Serial.print(getAdcValue(adcs, i));
            Serial.print(",");
        }
        Serial.println();
    }
}
#endif