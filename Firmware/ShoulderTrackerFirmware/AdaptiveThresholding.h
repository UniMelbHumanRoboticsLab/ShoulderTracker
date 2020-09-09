#include <Arduino.h>

#include <iterator.h>
/** ShoulderTracking device firmware
 * 
 * Copyright Vincent Crocher - Unimelb - 2016, 2020
 * License MIT license
 */
#include <vector.h>
#include <serstream.h>

//#define DEBUG
//#define MEMORY_DEBUG

#ifdef MEMORY_DEBUG
  typedef struct __freelist {
    size_t sz;
    struct __freelist *nx;
  } FREELIST;
   
  extern FREELIST *__flp;
  extern char *__brkval;
  
  size_t getMemoryUsed()
  {
    size_t used;
    FREELIST *fp;
   
  // __brkval=0 if nothing has been allocated yet
   
    if(__brkval==0)
      return 0;
   
  // __brkval moves up from __malloc_heap_start to
  // __malloc_heap_end as memory is used
   
    used=__brkval-__malloc_heap_start;
   
  // memory free'd by you is collected in the free list and
  // compacted with adjacent blocks. This, combined with malloc's
  // intelligent picking of candidate blocks drastically reduces
  // heap fragmentation. Anyway, since blocks in the free list
  // are available to you at no cost we need to take them off.
   
    for(fp=__flp;fp;fp=fp->nx)
      used-=fp->sz+sizeof(size_t);
   
    return used;
  }
  
  size_t getFreeMemory()
  {
    return (size_t)AVR_STACK_POINTER_REG-
           (size_t)__malloc_margin-
           (size_t)__malloc_heap_start-
           getMemoryUsed();
  }
#endif


unsigned char uchar_min(unsigned char a, unsigned char b){return (a<=b ? a:b);}
unsigned char uchar_max(unsigned char a, unsigned char b){return (a>=b ? a:b);}

#define NB_PTS 150 // (300 => 5min, 150 =>2min30s) WARNING: check memory capacity


//###################################################################################
//                            THRESHOLDING CLASS
//###################################################################################

class AdaptiveThresholding
{

  public:
    typedef unsigned int ValuesType; //Types of values stored by the class (will afect precision together with scale factor)
    std::vector<ValuesType> Ordered;
    std::vector<unsigned char> OrderedOrder;
    unsigned long int StorageT;
    
    float ScalingFactor;
    
    unsigned int tmpNbVal;
    double tmpMax;//tmpAvg;

    //Constructor: reserve vectors memory and init values
    AdaptiveThresholding(float scale=1)
    {
        Ordered.reserve(NB_PTS);
        OrderedOrder.reserve(NB_PTS);
        
        ScalingFactor=scale;
		
        Init();
    }
    
    //Reset everything: clear vectors and reinit values
    void Reset(float scale=1)
    {
        //Clear vectors
        Ordered.clear();
        OrderedOrder.clear();
  
        ScalingFactor=scale;
  		
        Init();
    }


    void Init()
    {
        //Init with two 0 values
        Ordered.push_back(0);
        Ordered.push_back(0);
        OrderedOrder.push_back(0);
        OrderedOrder.push_back(1);
      
        StorageT=micros();
        tmpNbVal=0;
        //tmpAvg=0;
        tmpMax=0;
    }
    
    //Getters/setters
    void SetScalingFactor(float scale){ScalingFactor=scale;}
    float GetScalingFactor(){return ScalingFactor;}


    //Add a value to storage
    void Store(float v)
    { 
      //If we are in the current second
      if(millis()-StorageT<1000 && millis()-StorageT>0)
      {
         tmpNbVal++;

         //Avg: keep it in float until storage
         //tmpAvg+=V;
         //Keep max value over last second
         tmpMax=fmax(tmpMax, v);
      }
      //When current second is over: store and reset
      else
      {
         //Compute average
         //tmpAvg/=(double)tmpNbVal;
         //tmpAvg*=ScalingFactor;
         //Compute scaled max value
         tmpMax*=ScalingFactor;
        
         //Store in memory
         #ifdef MEMORY_DEBUG
           Serial.print(getFreeMemory());Serial.println("B");
         #endif
         /*Debug
         Serial.print(millis());Serial.print("ms, ");Serial.print(tmpNbVal);Serial.print("sp, ");Serial.println(tmpAvg);*/
         Insert((ValuesType) tmpMax);
         
         //Reset orig time
         StorageT=millis();
         //Reset tmp values
         tmpMax=0;
         //tmpAvg=0;
         tmpNbVal=0;
      }
    }
    
    
    //Insert element in the ordered vector
    void Insert(ValuesType newVal)
    {
      #ifdef DEBUG
        Serial.print("Add");
      #endif
      //If vector already full
      if(Ordered.size()>=NB_PTS)
      {
        #ifdef DEBUG
          Serial.print(" FULL");
        #endif
        /*Debug
        Serial.print("erase ");Serial.println(OrderedOrder[0]);*/
        //Delete the oldest: its idx is the first value of OrderedOrder
        #ifdef DEBUG
          Serial.print(" Eject: ");
          Serial.print(OrderedOrder[0]);
          Serial.print(" (");
          Serial.print(Ordered[OrderedOrder[0]]);
          Serial.print(")  ");
        #endif
        Ordered.erase(Ordered.begin()+OrderedOrder[0]);
        
        //Update indexes values: all the ones above the deleted one are decreased by one
        for(int i=1; i<OrderedOrder.size(); i++)
        {
          if(OrderedOrder[i]>=OrderedOrder[0])
            OrderedOrder[i]--;
        }
        
        //Remove its index from the ordering order list
        OrderedOrder.erase(OrderedOrder.begin());
      }
      
      //Insert the most recent value in the list (sorted, using dichotomy):
      //start in the middle of the tab
      int pos=(int)(Ordered.size()/2.);
      //to begin increment is half of the position
      int inc=(int)(pos/2.);

      //Find the position of the new value
      while( pos<Ordered.size() && pos>-1 && (newVal>Ordered[pos] || newVal<Ordered[pos-1]) )
      {
        //if >, increment pos
        if( newVal>Ordered[pos] )
          pos+=inc;
        //otherwise decrement it
        else
          pos-=inc;

        //update increment (divide it by 2)
        inc=(int)(inc/2.)-1;
        if(inc<1)
          inc=1;

        //if we are adding the lowest value (and we know it <=> pos==0) then exit while
        if (pos==0 && newVal<=Ordered[pos])
          break;
      }
      #ifdef DEBUG
        Serial.print(" pos:");
        Serial.print(pos);
        Serial.print(" val:");
        Serial.println(newVal);
      #endif
  
      //insert element at the position just found
      std::vector<ValuesType>::iterator it;
      it = Ordered.begin();
      it+=pos;
      Ordered.insert(it, newVal);
      
      //Keep track of its position
      OrderedOrder.push_back(pos);
      //Update the other ones: all ones w/ higher value than pos should be incremented by 1
      for(int i=0; i<OrderedOrder.size()-1; i++)
      {
        if(OrderedOrder[i]>=pos)
          OrderedOrder[i]++;
      }
      /*Debug combo
      for(int i=0; i<Ordered.size(); i++)
      {
        Serial.print(Ordered[i]);Serial.print('(');Serial.print(OrderedOrder[i]);Serial.print(')');Serial.print(" ,");
      }
      Serial.println(" - ");*/
    }
      
      
    //Return threshold based on stored values and 
    // percentage
    float GetThreshold(int percent)
    {
      int idx=percent*Ordered.size()/100;
      //Use the values around to smooth the threshold when possible
      float thresh;
      /*if(idx<Ordered.size())
        thresh=(Ordered[idx-1]+Ordered[idx]+Ordered[idx+1])/3.;
      else*/
        thresh=Ordered[idx];
        
      
      #ifdef DEBUG
        Serial.print("Idx:");
        Serial.print(idx);
        Serial.print(" Thresh:");
        Serial.print(thresh);
        Serial.print(" Scaled:");
        Serial.println(thresh/ScalingFactor);
     #endif
 
      //Re-Scaling
      return thresh/ScalingFactor;
    }

    int GetNbPoints()
    {
      return Ordered.size();
    }

};
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
