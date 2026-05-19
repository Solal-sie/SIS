#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include "kiss_fft/kiss_fft.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SIGNAL_LENGTH 200

#define LIGHT_DEBUG 1

enum LightStatus {
	NO_LIGHT_DETECTED = 0,
	GOOD_LIGHT,
	FLICKER_LIGHT,
	BLINKING_LIGHT
};

struct LightAnalysisResult {
	LightStatus status;
	double dominant_freq;
	double dc_ratio;
	double low_freq_ratio;
	double high_freq_ratio;
};

inline const char* light_status_to_string(LightStatus s){
	switch(s){
    	case GOOD_LIGHT: 	return "good";
    	case FLICKER_LIGHT:  return "flicker";
    	case BLINKING_LIGHT: return "blinking";
    	default:         	return "unknown";
	}
}

inline double compute_mean(const std::vector<double>& x){
	if(x.empty()) return 0.0;

	double sum = 0.0;
	for(double v : x){
    	sum += v;
	}

	return sum / (double)x.size();
}

inline double compute_std(const std::vector<double>& x){
	if(x.size() < 2) return 0.0;

	double mean = compute_mean(x);
	double sum = 0.0;

	for(double v : x){
    	double d = v - mean;
    	sum += d * d;
	}

	return std::sqrt(sum / (double)x.size());
}

inline double compute_min(const std::vector<double>& x){
	if(x.empty()) return 0.0;
	return *std::min_element(x.begin(), x.end());
}

inline double compute_max(const std::vector<double>& x){
	if(x.empty()) return 0.0;
	return *std::max_element(x.begin(), x.end());
}

inline void preprocess_signal_hann(const std::vector<double>& raw_signal,
                               	std::vector<double>& processed_signal){

	processed_signal.resize(SIGNAL_LENGTH);

	double mean = compute_mean(raw_signal);

	for(int i = 0; i < SIGNAL_LENGTH; i++){

    	double centered = raw_signal[i] - mean;

    	double hann = 0.5 * (
        	1.0 - std::cos(
            	2.0 * M_PI * (double)i / (double)(SIGNAL_LENGTH - 1)
        	)
    	);

    	processed_signal[i] = centered * hann;
	}
}

inline void compute_fft_magnitude(const std::vector<double>& signal,
                              	std::vector<double>& magnitude){

	kiss_fft_cfg cfg = kiss_fft_alloc(SIGNAL_LENGTH, 0, NULL, NULL);

	kiss_fft_cpx cx_in[SIGNAL_LENGTH];
	kiss_fft_cpx cx_out[SIGNAL_LENGTH];

	for(int i = 0; i < SIGNAL_LENGTH; i++){
    	cx_in[i].r = signal[i];
    	cx_in[i].i = 0.0;
	}

	kiss_fft(cfg, cx_in, cx_out);

	magnitude.resize(SIGNAL_LENGTH / 2);

	for(int k = 0; k < SIGNAL_LENGTH / 2; k++){
    	magnitude[k] = std::sqrt(
        	cx_out[k].r * cx_out[k].r +
        	cx_out[k].i * cx_out[k].i
    	);
	}

	free(cfg);
}

inline LightAnalysisResult analyze_light_signal(const std::vector<double>& raw_signal,
                                            	double dt){

	LightAnalysisResult result;

	result.status = NO_LIGHT_DETECTED;
	result.dominant_freq = 0.0;
	result.dc_ratio = 0.0;
	result.low_freq_ratio = 0.0;
	result.high_freq_ratio = 0.0;

	if(raw_signal.size() != SIGNAL_LENGTH || dt <= 0.0){
    	return result;
	}

	double raw_mean = compute_mean(raw_signal);
	double raw_std = compute_std(raw_signal);
    
	#if LIGHT_DEBUG
	printf("\n============================\n");
	printf("NEW LIGHT ANALYSIS\n");
	printf("============================\n");
    
	printf("raw_mean  	= %.6f\n", raw_mean);
	printf("raw_std   	= %.6f\n", raw_std);
	printf("relative_std  = %.6f\n", raw_std / raw_mean);
    
	printf("raw_min   	= %.6f\n", compute_min(raw_signal));
	printf("raw_max   	= %.6f\n", compute_max(raw_signal));
	#endif

	if(raw_mean <= 1e-9){
    	return result;
	}

	std::vector<double> processed_signal;
	preprocess_signal_hann(raw_signal, processed_signal);
    
	#if LIGHT_DEBUG
	printf("\nFirst 10 processed samples:\n");
    
	for(int i = 0; i < 10; i++){
    	printf("[%d] %.6f\n", i, processed_signal[i]);
	}
	#endif

	std::vector<double> mag;
	compute_fft_magnitude(processed_signal, mag);
    
	#if LIGHT_DEBUG
	printf("\nFirst FFT bins:\n");
    
	for(int k = 0; k < 10; k++){
    
    	double freq =
        	(double)k * (1.0 / dt) / (double)SIGNAL_LENGTH;
    
    	printf(
        	"bin=%d | freq=%.3f Hz | mag=%.6f\n",
        	k,
        	freq,
        	mag[k]
    	);
	}
	#endif
    
	double fs = 1.0 / dt;

	double total_energy = 0.0;
	double low_energy = 0.0;
	double high_energy = 0.0;

	int k_dom = 1;
	double max_mag = 0.0;

	for(size_t k = 1; k < mag.size(); k++){

    	double freq = (double)k * fs / (double)SIGNAL_LENGTH;

    	if(freq < 0.3){
        	continue;
    	}

    	total_energy += mag[k];

    	if(mag[k] > max_mag){
        	max_mag = mag[k];
        	k_dom = (int)k;
    	}

    	if(freq >= 0.3 && freq < 2.0){
        	low_energy += mag[k];
    	}
    	else if(freq >= 2.0){
        	high_energy += mag[k];
    	}
	}

	double relative_std = raw_std / raw_mean;
    
	#if LIGHT_DEBUG
	printf("\nEnergy analysis:\n");
    
	printf("total_energy 	= %.6f\n", total_energy);
	printf("low_energy   	= %.6f\n", low_energy);
	printf("high_energy  	= %.6f\n", high_energy);
    
	printf("low_freq_ratio   = %.6f\n",
       	low_energy / total_energy);
    
	printf("high_freq_ratio  = %.6f\n",
       	high_energy / total_energy);
	#endif

	if(total_energy <= 1e-9){
    
    	result.status = GOOD_LIGHT;
    	result.dc_ratio = 1.0;
    
    	#if LIGHT_DEBUG
    	printf("\nClassification reason:\n");
    	printf("Signal almost constant.\n");
    	printf("=> GOOD_LIGHT\n");
    	#endif
    
    	return result;
	}

	result.dominant_freq = (double)k_dom * fs / (double)SIGNAL_LENGTH;
	result.low_freq_ratio = low_energy / total_energy;
	result.high_freq_ratio = high_energy / total_energy;
	result.dc_ratio = 1.0 / (1.0 + relative_std);
    
	#if LIGHT_DEBUG
	printf("\nFinal ratios:\n");
    
	printf("dc_ratio     	= %.6f\n", result.dc_ratio);
	printf("low_freq_ratio   = %.6f\n", result.low_freq_ratio);
	printf("high_freq_ratio  = %.6f\n", result.high_freq_ratio);
    
	printf("dominant_freq	= %.6f Hz\n",
       	result.dominant_freq);
	#endif
    
	if(result.high_freq_ratio > 0.65 && relative_std > 0.005){
    	#if LIGHT_DEBUG
    	printf("\nDetected HIGH frequency dominance.\n");
    	printf("=> FLICKER_LIGHT\n");
    	#endif
    	result.status = FLICKER_LIGHT;
	}
	else if(result.low_freq_ratio > 0.65 && relative_std > 0.02){
    	#if LIGHT_DEBUG
    	printf("\nDetected LOW frequency dominance.\n");
    	printf("=> BLINKING_LIGHT\n");
    	#endif
    	result.status = BLINKING_LIGHT;
	}
	else{
    
    	#if LIGHT_DEBUG
    	printf("\nNo dominant oscillation detected.\n");
    	printf("=> GOOD_LIGHT\n");
    	#endif
    
    	result.status = GOOD_LIGHT;
	}
	#if LIGHT_DEBUG
	printf("\nFINAL CLASSIFICATION = %s\n",
       	light_status_to_string(result.status));
    
	printf("============================\n\n");
	#endif
	return result;
}

inline LightStatus classify_light_signal(const std::vector<double>& raw_signal,
                                     	double dt){
	return analyze_light_signal(raw_signal, dt).status;
}

inline LightStatus classify_light_fft(const double* raw_signal,
                                  	double dt,
                                  	int n){

	if(raw_signal == nullptr || dt <= 0.0 || n < SIGNAL_LENGTH){
    	return NO_LIGHT_DETECTED;
	}

	std::vector<double> signal(SIGNAL_LENGTH);

	for(int i = 0; i < SIGNAL_LENGTH; i++){
    	signal[i] = raw_signal[i];
	}

	return classify_light_signal(signal, dt);
}
