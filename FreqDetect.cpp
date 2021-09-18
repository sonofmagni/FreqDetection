/* -- FreqDet Algorithm --

The MIT License (MIT)

Copyright (c) 2021 Karl Thorkildsen / Thor Bass

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

    // find peaks using moving average
    // collect intervals
    // average n intervals
    // throw out bad intervals
    // average remaining intervals

    // this is optimized for electric bass but is generally more stable for higher frequency instruments
    
    Copyright Karl Thorkildsen / Thor Bass
    If you use this code even in part please credit me with the 
 */

#include "daisysp.h"
#include "daisy_petal.h"

// turn off compiler optimizations
#pragma GCC push_options
#pragma GCC optimize ("O0")

// could be smaller
#define MAX_SIZE (48000 / 2)

using namespace daisysp;
using namespace daisy;

static DaisySeed seed;
static DaisyPetal petal;

int                 pos = 0;
int                 lockPos = 0;
float DSY_SDRAM_BSS buf[MAX_SIZE];
float DSY_SDRAM_BSS buf2[MAX_SIZE];
int                 mod    = MAX_SIZE;
int                 len    = 0;
float               drywet = 0;
bool                res    = false;
float               inputGain = 0;

    // moved a bunch of what should be locals here because debugging only gives correct values for global fields
    float sampleSum = 0;
    int sampleIndex = 0;
    int halfPeriodSampleCount = 0;
    int lengthNonZero = 1;


void ResetBuffer();
void Controls();

void NextSamples(float&                               output,
                 AudioHandle::InputBuffer in,
                 size_t                               i);

struct LowPassFilter
{
    Svf   filt;
    float amp;

    void Init(float samplerate, float freq)
    {
        filt.Init(samplerate);
        filt.SetRes(0.01);
        filt.SetDrive(.002);
        filt.SetFreq(freq);
    }

    float Process(float in)
    {
        filt.Process(in);
        return filt.Low();
    }
};
struct HighPassFilter
{
    Svf   filt;
    float amp;

    void Init(float samplerate, float freq)
    {
        filt.Init(samplerate);
        filt.SetRes(0.01);
        filt.SetDrive(.002);
        filt.SetFreq(freq);
    }

    float Process(float in)
    {
        filt.Process(in);
        return filt.High();
    }
};

LowPassFilter lpFilter;
HighPassFilter hpFilter;

static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    float output = 0;

    for(size_t i = 0; i < size; i ++)
    {
        NextSamples(output, in, i);

        // left output
        out[0][i] = output;
    }
}

int aveBufPtr = 0;
float movAve = 0;
#define AVE_BUF_SIZE 8
float aveBuffer[AVE_BUF_SIZE];

// Return the average of the recent samples
float CalcMovAve()
{
    float sum = 0;
    for (int ptr = 0; ptr < AVE_BUF_SIZE; ptr++)
    {
        sum = sum + aveBuffer[ptr];
    }
    return sum / float(AVE_BUF_SIZE);
}

// Add a sample to the collection of recent samples
void AddMovAveValue(float val)
{
    if (aveBufPtr >= AVE_BUF_SIZE) aveBufPtr = 0;
    aveBuffer[aveBufPtr] = val;
    aveBufPtr++;
}

#define ZEROCROSS 0.005

float GetPeakPeakInterval()
{
    // avoid lockpos
    // find zero point
    // find peaks using moving average
    // collect intervals
    // average n intervals
    // throw out bad intervals
    // average remaining intervals

    sampleSum = 0;
    sampleIndex = 0;
    int sampleStart = 0;
    bool midStart = false;
    #define PEAK_SAMPLE_COUNT 32
    int peakSamples[PEAK_SAMPLE_COUNT];
    int peakSampleIndex = 0;
    int peakSamplePeriods[PEAK_SAMPLE_COUNT];
    #define SCAN_LENGTH 4800
 
    if (lockPos < MAX_SIZE * 0.4)
    {
        sampleStart = MAX_SIZE / 2;     // this still gives us 6000 samples to look at which would be 5 cycles at 40hz
        midStart = true;
    }

    // Preload the moving average buffer
    for (int fill = 0; fill < AVE_BUF_SIZE; fill++)
    {
        AddMovAveValue(buf2[sampleStart]);
        sampleStart++;
    }

    // TODO: messy
    int searchEnd = MAX_SIZE / 3;
    if (midStart) searchEnd = MAX_SIZE;

    // first we need to understand how big a peak is
    // this is so we can reject peaks that are not significant
    float peakValue = 0.0;
    for (sampleIndex = sampleStart; sampleIndex < sampleStart + 1200; sampleIndex++) // at least one peak at 40hz
    {
        AddMovAveValue(buf2[sampleIndex]);
        float thisPeak = CalcMovAve();
        if (peakValue < thisPeak)
        {
            peakValue = thisPeak;
        }
    }

    // don't try to process tiny signals
    if (peakValue < 0.005) return 100000;  // almost 0hz

    // reject peaks that are not 80% of the one we found
    peakValue = peakValue * 0.9;

    // Preload the moving average buffer - TODO: refactor this
    for (int fill = 0; fill < AVE_BUF_SIZE; fill++)
    {
        AddMovAveValue(buf2[sampleStart]);
        sampleStart++;
    }
    
    peakSampleIndex = 0;
    bool foundThisPeak = false;
    // collect peaks using moving average
    for (sampleIndex = sampleStart; sampleIndex < searchEnd; sampleIndex++)
    {
        AddMovAveValue(buf2[sampleIndex]);
        // done with the previous peak
        if (CalcMovAve() < peakValue * -0.2) foundThisPeak = false;

        if (CalcMovAve() > peakValue && !foundThisPeak)
        {
            // continue looking until the peak value starts to decrease (the peak of the peak)
            float nearPeak = CalcMovAve();
            sampleIndex++;
            for (; sampleIndex < searchEnd; sampleIndex++)
            {
                AddMovAveValue(buf2[sampleIndex]);
                float newPeak = CalcMovAve();
                if (nearPeak <= newPeak)
                {
                    nearPeak = newPeak;
                }
                else //if (nearPeak > newPeak)
                {
                    peakSamples[peakSampleIndex] = sampleIndex;
                    peakSampleIndex++;
                    foundThisPeak = true;
                    break;
                }
            }
            if (peakSampleIndex >= PEAK_SAMPLE_COUNT) break;
        }
    }

    // don't try to process with too few samples
    if (peakSampleIndex < 4)
    {
        petal.seed.PrintLine("Not enough samples");
        return 100000;  // almost 0hzP
    }

    peakSampleIndex--;

    int numPeakSamples = peakSampleIndex + 1;
    int numPeriods = peakSampleIndex;
    // now we have peakSamples[] with the positive peaks and numPeakSamples is the count
    // convert the array of peak sample pointers into an array of periods
    for (peakSampleIndex = 1; peakSampleIndex < numPeakSamples; peakSampleIndex++)
    {
        peakSamplePeriods[peakSampleIndex-1] = peakSamples[peakSampleIndex] - peakSamples[peakSampleIndex-1];
    }

    // average the periods
    int totalPeriod = 0;
    int averagePeriod = 0;
    for (int perIndex = 0; perIndex < numPeriods; perIndex++)
    {
        totalPeriod = totalPeriod + peakSamplePeriods[perIndex];
    }

    averagePeriod = totalPeriod / numPeriods;

    int goodPerIndex = 0;
    int goodPeriods[PEAK_SAMPLE_COUNT];
    // throw out bad values
    for (int perIndex = 0; perIndex < numPeriods; perIndex++)
    {
        if (peakSamplePeriods[perIndex] < averagePeriod * 1.6 && peakSamplePeriods[perIndex] > averagePeriod * 0.65)
        {
            goodPeriods[goodPerIndex] = peakSamplePeriods[perIndex];
            goodPerIndex++;
        }
    }

    int goodPerSum = 0;
    int goodPerCount = goodPerIndex;
    for (goodPerIndex = 0; goodPerIndex < goodPerCount; goodPerIndex++)
    {
        goodPerSum += goodPeriods[goodPerIndex];
    }
    int goodPerAverage = goodPerSum / goodPerCount;     // TODO: messy

    return goodPerAverage * 0.0000208333;
}

bool failed = false;

// this is to show whether the note played is near the note expected
void SetEncoderLights(int expectedF, int actualF)
{
    // initialize lights all red
    for(size_t i = 0; i < petal.RING_LED_LAST; i++)
    {
        petal.SetRingLed(static_cast<DaisyPetal::RingLed>(i), 0.5, 0, 0);   //red, green, blue
    }

    if (actualF > 20 && actualF < 600)
    {
        // find which to turn green
        if (actualF < expectedF * 1.1 && actualF > expectedF * 0.9)
        {
            petal.SetRingLed(static_cast<DaisyPetal::RingLed>(1), 0, 0, 0.5);   //red, green, blue
            petal.SetRingLed(static_cast<DaisyPetal::RingLed>(2), 0, 0, 0.5);   //red, green, blue
        }
        else if (actualF > expectedF && actualF < expectedF * 1.4)
        {
            petal.SetRingLed(static_cast<DaisyPetal::RingLed>(3), 0, 0, 0.5);   //red, green, blue
        }
        else if (actualF < expectedF && actualF > expectedF * 0.6)
        {
            petal.SetRingLed(static_cast<DaisyPetal::RingLed>(0), 0, 0, 0.5);   //red, green, blue
        }
        else if (actualF > expectedF)
        {
            petal.SetRingLed(static_cast<DaisyPetal::RingLed>(4), 0, 0, 0.5);   //red, green, blue
        }
        else if (actualF < expectedF)
        {
            petal.SetRingLed(static_cast<DaisyPetal::RingLed>(7), 0, 0, 0.5);   //red, blue, green
        }
    }
}

int main(void)
{
    // initialize petal hardware and oscillator daisysp module
    petal.Init();
    ResetBuffer();

    petal.seed.StartLog(false);
    System::Delay(500);

    // start callback
    petal.StartAdc();
    petal.StartAudio(AudioCallback);

    System::Delay(500);

    lpFilter.Init(48000, 1000);
    hpFilter.Init(48000, 10);

    while(1)
    {
        petal.ProcessAnalogControls();
        inputGain = petal.GetKnobValue(petal.KNOB_1) * 4;
        lockPos = pos;

        // very solid with electric bass
        float period = GetPeakPeakInterval();

        int frequency = 1 / period;
        float ff = 1/period;
        petal.seed.PrintLine("freq %d\n", int(ff*1000));

        SetEncoderLights(83, frequency);
        petal.UpdateLeds();

        System::Delay(8);   // sleep
        System::Delay(8);   // sleep
    }
}

//Resets the buffer
void ResetBuffer()
{
    pos   = 0;
    len   = 0;
    for(int i = 0; i < mod; i++)
    {
        buf[i] = 0;
        buf2[i] = 0;
    }
    mod = MAX_SIZE;
}

void WriteBuffer(AudioHandle::InputBuffer in, size_t i)
{
    buf[pos] = in[0][i] * inputGain;
    buf2[pos] = lpFilter.Process(buf[pos]);
}

void NextSamples(float&                   output,
                 AudioHandle::InputBuffer in,
                 size_t                   i)
{
    if(true)
    {
        WriteBuffer(in, i);
    }

    output = buf[pos];

    pos++;
    if (pos == MAX_SIZE)
    {
        pos = 0;
    }
}
