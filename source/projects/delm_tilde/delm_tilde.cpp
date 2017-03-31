/****************************************************
 *   This code is explicated in Chapter 4 of        *
 *   "Designing Audio Objects for Max/MSP and Pd"   *
 *   by Eric Lyon.                                  *
 ****************************************************/

/* Required header files */


#include "c74_msp.h"

#include <vector>
#include <iostream>
#include <memory>

#include "MultiDelayLine.h"

using namespace c74::max;

#define MAX_DELAYS 64

/* The class pointer */

static t_class *delm_class;

/* The object structure */

struct t_delm {
  t_pxobject obj; // the Max/MSP object
  float sr; // sampling rate
  float maximum_delay_time; // maximum delay time
  long delay_length; // length of the delay line in samples
  
  float delay_time; // current delay time
  
  float del_num;  // number of delay times input
  float feedback; // feedback multiplier
  float numChan; // feedback multiplier
  
  long write_ptr; // write point in delay line

  short delaytime_connected; // inlet connection status
  short feedback_connected; // inlet connection status
  
  std::unique_ptr<MultiDelayLine> delayLine;
};

/* Function prototypes */

void *delm_new(t_symbol *s, short argc, t_atom *argv);
t_int *delm_perform_denormals(t_int *w);
void delm_assist(t_delm *x, void *b, long msg, long arg, char *dst);
void delm_free(t_delm *x);
void delm_float(t_delm *x, double f);
void delm_list(t_delm *x, t_symbol *msg, short argc, t_atom *argv);

t_max_err a_channel_set(t_delm *x, t_object *attr, long argc, t_atom *argv);

void delm_perform64(t_delm *x, t_object *dsp64, double **ins,
                    long numins, double **outs,long numouts, long n,
                    long flags, void *userparam);
void delm_dsp64(t_delm *x, t_object *dsp64, short *count, double sr, long n, long flags);


/* The main() function */

void ext_main(void *r)
{
  delm_class = class_new("delm~",(method)delm_new,(method)delm_free,sizeof(t_delm),0,A_GIMME,0);
  class_addmethod(delm_class, (method)delm_dsp64, "dsp64", A_CANT, 0);
  class_addmethod(delm_class, (method)delm_assist, "assist", A_CANT, 0);
  class_addmethod(delm_class, (method)delm_float, "float", A_FLOAT, 0);
  class_addmethod(delm_class, (method)delm_list, "list", A_GIMME, 0);
  class_dspinit(delm_class);
  
  CLASS_ATTR_FLOAT             (delm_class, "channels", 0, t_delm, numChan);          // Assign @channel attribute to t_delm->numChan
  //CLASS_ATTR_ACCESSORS		(delm_class, "channels", NULL, a_channel_set);          // For seperate setter function
  CLASS_ATTR_LABEL            (delm_class, "channels", NULL, "Number of Channels");   // Show Attribute Name in Inspector
  //CLASS_ATTR_ORDER            (delm_class, "channels", 0, "1");                     // Order number for display
  
  class_register(CLASS_BOX, delm_class);
  cpost("delm~ from Thomas Mayr");
}

/* The new instance routine */
void *delm_new(t_symbol *s, short argc, t_atom *argv)
{
  int i;
  
  // Maximum delay init
  float delmax = 1000.0;
  
  // Default: 2 Channel
  float numChan = 2;
  
  // Instantiate a new delm~ object
  t_delm *x = (t_delm*) object_alloc(delm_class);
  
  // Store initial channel number
  x->numChan = numChan;
  
  attr_args_process(x, argc, argv);   // For Attributes
  // Create three signal inlets
  dsp_setup(&x->obj, x->numChan);
  
  
  // Create multiple Outlets
  for (size_t i = 0; i < x->numChan; i++) {
    outlet_new((t_object *)x, "signal"); 		// signal outlet (note "signal" rather than NULL)
  }
  
  //force independent signal vectors
  x->obj.z_misc |= Z_NO_INPLACE;
  
  x->delayLine.reset(new MultiDelayLine);
  
  // Get samplerate
  x->sr = sys_getsr();
  
  // Set the  maximum delay if necessary
  if(delmax <= 0){
    delmax = 1000.0;
  }
  
  /* Convert from ms to secs */
  x->maximum_delay_time = delmax * 0.001;
  
  float l = x->sr * x->maximum_delay_time + 1.0f;
  unsigned int const t = 1U << ((*(unsigned int *)&l >> 23) - 0x7f);
  /* Allocate memory for the delay line */
  x->delay_length = t << (t < l);;
  
  /*
  for (t_delm::DelayLine& del : x->delay_lines) {
    del.resize(x->delay_length, 0.0f);
  }*/

  /* Initialize the write index */
  x->write_ptr = 0;
  
  
  /* Return a pointer to the object */
  return x;
}

/* The free memory routine */

void delm_free(t_delm *x)
{
  /* We must call dsp_free() before freeing any dynamic memory
   allocated for the external. This removes the object from the
   Max/MSP DSP chain. */
  
  dsp_free((t_pxobject *) x);
  
  /* Now we safely free the delay line memory */
  //sysmem_freeptr(x->delay_line);
}

// The list method for the delay times
void delm_list (t_delm *x, t_symbol *msg, short argc, t_atom *argv)
{
  short i;
  int del_num = 0;
//  float *dels = x->delay_times;
//  if(argc == x->numChan){
//    for (i=0; i < argc; i++) {
//      int a = atom_getfloat(argv + i);
//      if(a > 0 || (a*0.001 < x->maximum_delay_time)){     // Check if input delay times are within 0 and max delay time
//        dels[del_num++] = a;
//      }
//      else{
//        cpost("delm~: delay time not in range");
//      }
//    }
//    x->del_num = del_num;
//  }
//  else
//  {
//    cpost("delm~: illegal number of delays");
//  }
}

// The float method
void delm_float(t_delm *x, double f)
{
  /* Programmatically determine the calling inlet */
  int inlet = ((t_pxobject*)x)->z_in;
  
  /* Select response based on which inlet received a float */
  switch(inlet){
      
      /* The second inlet is the delay time */
    case 1:
      if(f < 0.0 ||
         f > x->maximum_delay_time * 1000.0){
        object_post(reinterpret_cast<t_object*>(x), "delm~: illegal delay: %f reset to 1 ms", f);
      } else {
        x->delay_time = f;
      }
      break;
      
      /* The third inlet is the feedback factor */
    case 2:
      x->feedback = f;
      break;
  }
}

// Attribute set function if neccessary
t_max_err a_channel_set(t_delm *x, t_object *attr, long argc, t_atom *argv)
{
  //if(argc && argv)
  //{
  x->numChan = atom_getfloat(argv);
  //}
  return MAX_ERR_NONE;
}

// The perform routine
void delm_perform64(t_delm *x, t_object *dsp64, double **ins,
                    long numins, double **outs,long numouts, long n,
                    long flags, void *userparam)
{
  x->delayLine->process(ins, outs);
}

// The assist method
void delm_assist(t_delm *x, void *b, long msg, long arg, char *dst)
{
  if (msg == ASSIST_INLET) {
    switch (arg) {
      case 0: sprintf(dst,"(signal) Input"); break;
      case 1: sprintf(dst,"(signal) Delay Time"); break;
      case 2: sprintf(dst,"(signal) Feedback"); break;
    }
  } else  {
    //sprintf(dst,"(signal) %d", i);
    sprintf(dst,"(signal) Output %ld", arg+1);
  }
}

// The DSP method
void delm_dsp64(t_delm *x, t_object *dsp64, short *count, double sr, long n, long flags)
{
  //post("delay_times %.0f %.0f %.0f %.0f \n", x->delay_times[0],x->delay_times[1],x->delay_times[2],x->delay_times[3]);
  //post("idelay %.0ld %.0ld %.0ld %.0ld \n", x->idelay[0],x->idelay[1],x->idelay[2],x->idelay[3]);
  /* Store inlet connection states */
  
  int i;
  
  /* Store inlet connection states */
  
  x->delaytime_connected = count[1];
  x->feedback_connected = count[2];
  
  /* Reset delayline if the sampling rate has changed */
  
  x->delayLine->init(sr, 2, n, x->numChan);

  
  if(x->sr != sr){
    x->sr = sr;
    
    float l = x->sr * x->maximum_delay_time + 1.0f;
    unsigned int const t = 1U << ((*(unsigned int *)&l >> 23) - 0x7f);
    /* Allocate memory for the delay line */
    x->delay_length = t << (t < l);;
    
    x->write_ptr = 0;
  }
  else {
  }
  
  // x->mixBuffer.resize(x->numChan);
  
  //for (auto& buf : x->mixBuffer) {
  //  buf.resize(n, 0.0f);
  //}
  
  /* Local variables */
  const long delay_length = x->delay_length;        // Length of the Delay line
  const float srms = sr / 1000.0f;                   // calculation from milliseconds to seconds
  // float out_sample;                           // output for every output channel
  
  /* Add vdelay~ to the Max/MSP DSP chain */
  
  
  object_method_direct(void, (t_object*, t_object*, t_perfroutine64, long, void*),
                       dsp64, gensym("dsp_add64"), (t_object*)x, (t_perfroutine64)delm_perform64, 0, NULL);
  
}
