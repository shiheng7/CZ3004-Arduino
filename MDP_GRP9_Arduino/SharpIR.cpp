/*
	SharpIR

	Arduino library for retrieving distance (in cm) from the analog GP2Y0A21Y and GP2Y0A02YK

	From an original version of Dr. Marcal Casas-Cartagena (marcal.casas@gmail.com)     
	
    Version : 1.0 : Guillaume Rico
    + Remove average and use median
    + Definition of number of sample in .h
    + Define IR pin as input

	https://github.com/guillaume-rico/SharpIR
    
    Original comment from Dr. Marcal Casas-Cartagena :
   The Sharp IR sensors are cheap but somehow unreliable. I've found that when doing continous readings to a
   fix object, the distance given oscilates quite a bit from time to time. For example I had an object at
   31 cm. The readings from the sensor were mainly steady at the correct distance but eventually the distance
   given dropped down to 25 cm or even 16 cm. That's quite a bit and for some applications it is quite
   unacceptable. I checked the library http://code.google.com/p/gp2y0a21yk-library/ by Jeroen Doggen
   (jeroendoggen@gmail.com) and what the author was doing is to take a bunch of readings and give an average of them

   The present library works similary. It reads a bunch of readings (avg), it checks if the current reading
   differs a lot from the previous one (tolerance) and if it doesn't differ a lot, it takes it into account
   for the mean distance.
   The distance is calculated from a formula extracted from the graphs on the sensors datasheets
   After some tests, I think that a set of 20 to 25 readings is more than enough to get an accurate distance
   Reading 25 times and return a mean distance takes 53 ms. For my application of the sensor is fast enough.
   This library has the formulas to work with the GP2Y0A21Y and the GP2Y0A02YK sensors but exanding it for
   other sensors is easy enough.

*/

/*
 * This is a variant on the original SharpIR file with optimisations added. Instead of using a single O(n^2) sort to obtain the median, 
 * this uses a median of medians to approximate the true value and return it. Effectively, this can halve sensor return time.
 * 
 * Author: Archery2000
 */
#ifdef Arduino
  #include "Arduino.h"
#elif defined(SPARK)
  #include "Particle.h"
  #include "math.h"
#endif
#include "SharpIR.h"

// Initialisation function
//  + irPin : is obviously the pin where the IR sensor is attached
//  + sensorModel is a int to differentiate the two sensor models this library currently supports:
//    1080 is the int for the GP2Y0A21Y and 20150 is the int for GP2Y0A02YK. The numbers reflect the
//    distance range they are designed for (in cm)
SharpIR::SharpIR(int irPin, int sensorModel) {
  
    _irPin=irPin;
    _model=sensorModel;
    
    // Define pin as Input
    pinMode (_irPin, INPUT);
    
    #ifdef ARDUINO
      analogReference(DEFAULT);
    #endif
}

// Sort an array
void SharpIR::sort(double a[], int size) {
    int t;
    bool flag;
    for(int i=0; i<(size-1); i++) {
        flag = true;
        for(int o=0; o<(size-(i+1)); o++) {
            if(a[o] > a[o+1]) {
                t = a[o];
                a[o] = a[o+1];
                a[o+1] = t;
                flag = false;
            }
        }
        if (flag) break;
    }
}

// Read distance and compute it
double SharpIR::distance() {

    double ir_val[NB_SAMPLE] = {};
    double distanceCM;

    for (int i=0; i<NB_SAMPLE; i++){
        // Read analog value
        ir_val[i] = analogRead(_irPin);
    }
    
    // Use median of medians to get approximate value
    if (_model==1080) {
        
        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
        #ifdef ARDUINO
          distanceCM = 27.728 * pow(map(medianOfMedians(ir_val, NB_SAMPLE), 0, 1023, 0, 5000)/1000.0, -1.2045);
        #elif defined(SPARK)
          distanceCM = 27.728 * pow(map(medianOfMedians(ir_val, NB_SAMPLE), 0, 4095, 0, 5000)/1000.0, -1.2045);
        #endif

    } else if (_model==20150){

        // Previous formula used by  Dr. Marcal Casas-Cartagena
        // puntualDistance=61.573*pow(voltFromRaw/1000, -1.1068);
        
        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
        #ifdef ARDUINO
          distanceCM = 60.374 * pow(map(medianOfMedians(ir_val, NB_SAMPLE), 0, 1023, 0, 5000)/1000.0, -1.16);
        #elif defined(SPARK)
          distanceCM = 60.374 * pow(map(medianOfMedians(ir_val, NB_SAMPLE), 0, 4095, 0, 5000)/1000.0, -1.16);
        #endif
    }

    return distanceCM;
}

double SharpIR::medianOfMedians(double a[], int size){
  int numMedians = size / 5;
  double medians[numMedians];
  for(int i = 0; i < numMedians; i++){
    partialSort(a, i * 5, i * 5 + 4);
    medians[i] = a[i * 5 + 2];
  }
  sort(medians, numMedians);
  return medians[numMedians/2];
}

// Sort a partial array
void SharpIR::partialSort(double a[], int min, int max) {
    int t;
    bool flag;
    for(int i=min; i<max; i++) {
        flag = true;
        for(int o=min; o<(max-i); o++) {
            if(a[o] > a[o+1]) {
                t = a[o];
                a[o] = a[o+1];
                a[o+1] = t;
                flag = false;
            }
        }
        if (flag) break;
    }
}
