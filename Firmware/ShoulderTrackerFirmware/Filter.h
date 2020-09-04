#define FILT_ORDER 1

class Filter
{
  public: 
    Filter() 
    {
      for(int k=0; k<FILT_ORDER-1; k++)
      {
        x[k]=0;
        y[k]=0;
      }
    }
  
    float filter(float xx)
    {
      //Save values
      for(int k=0; k<FILT_ORDER-1; k++)
      {
        x[k]=x[k+1];
        y[k]=y[k+1];
      }
      x[FILT_ORDER-1]=xx;
  
      //Apply filter
      y[FILT_ORDER-1] = 0;
      for(int k=0; k<FILT_ORDER+1; k++)
        y[FILT_ORDER-1] += b[k] * x[FILT_ORDER-1-k];

      for(int k=1; k<FILT_ORDER+1; k++)
        y[FILT_ORDER-1] -= a[k] * y[FILT_ORDER-1-k];

      y[FILT_ORDER-1] /= a[0];
  
      return y[FILT_ORDER-1];
    }

  private: 
    float a[FILT_ORDER+1]={1.0000, -0.9391};
    float b[FILT_ORDER+1]={0.9695, -0.9695};
    float x[FILT_ORDER], y[FILT_ORDER];
};
