#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <alsa/asoundlib.h>
#include <math.h>
#include <errno.h>

#define DEBUG 0

static char *device = "plughw:0,0";                  /* playback device */
static snd_pcm_format_t format = SND_PCM_FORMAT_S16; /* sample format */
static unsigned int rate = 44100;                    /* stream rate */
static unsigned int channels = 2;                    /* count of channels */
static unsigned int buffer_time = 500000;            /* ring buffer length in us */
static unsigned int period_time = 100000;            /* period time in us */
static double freq = 220;                            /* sinusoidal wave frequency in Hz */
static snd_pcm_sframes_t buffer_size;
static snd_pcm_sframes_t period_size;
static snd_output_t *output = NULL;
static snd_pcm_status_t *status;

static void generate_sine(const snd_pcm_channel_area_t *areas, 
			  snd_pcm_uframes_t offset,
			  int count, double *_phase) {
  static double max_phase = 2. * M_PI;
  double phase = *_phase;
  double step = max_phase*freq/(double)rate;
  unsigned char *samples[channels];
  int steps[channels];
  int format_bits = snd_pcm_format_width(format);
  unsigned int maxval = (1 << (format_bits - 1)) - 1;
  int bps = format_bits / 8;  /* bytes per sample */
  // int phys_bps = snd_pcm_format_physical_width(format) / 8;
  /* verify and prepare the contents of areas */
  for(unsigned int chn = 0; chn < channels; chn++) {
    if((areas[chn].first % 8) != 0) {
      printf("areas[%u].first == %u, aborting...\n", chn, areas[chn].first);
      exit(EXIT_FAILURE);
    }
    samples[chn] = /*(signed short *)*/(((unsigned char *)areas[chn].addr) + (areas[chn].first / 8));
    if((areas[chn].step % 16) != 0) {
      printf("areas[%u].step == %u, aborting...\n", chn, areas[chn].step);
      exit(EXIT_FAILURE);
    }
    steps[chn] = areas[chn].step / 8;
    samples[chn] += offset * steps[chn];
  }
  /* fill the channel areas */
  while(count-- > 0) {
    int res;
    res = sin(phase) * maxval;
    for(unsigned int chn = 0; chn < channels; chn++) {
      // Generate data in little endian: LSB at lowest address
      for(int i = 0; i < bps; i++)
	*(samples[chn] + i) = (res >> i * 8) & 0xff;
      samples[chn] += steps[chn];
    }
    phase += step;
    if (phase >= max_phase)
      phase -= max_phase;
  }
  *_phase = phase;
}

struct async_private_data {
  signed short *samples;
  snd_pcm_channel_area_t *areas;
  double phase;
};
static void transfer_callback(snd_async_handler_t *ahandler) {
  snd_pcm_t *handle = snd_async_handler_get_pcm(ahandler);
  struct async_private_data *data = snd_async_handler_get_callback_private(ahandler);
  signed short *samples = data->samples;
  snd_pcm_channel_area_t *areas = data->areas;
  snd_pcm_sframes_t avail;
  int err;

  if((avail = snd_pcm_avail_update(handle)) < 0) {
    if(avail == -EPIPE) {
      printf("Number of frames ready failed: %s\n", snd_strerror(avail));
      return;
    }
  }
  while(avail >= period_size) {
    generate_sine(areas, 0, period_size, &data->phase);
    err = snd_pcm_writei(handle, samples, period_size);
    if(err < 0) {
      printf("Write error: %s\n", snd_strerror(err));
      exit(EXIT_FAILURE);
    }
    if(err != period_size) {
      printf("Write error: written %i expected %li\n", err, period_size);
      exit(EXIT_FAILURE);
    }
    avail = snd_pcm_avail_update(handle);
  }
}
// Async mode transfer loop
static int transfer_loop(snd_pcm_t *handle,
		      signed short *samples,
		      snd_pcm_channel_area_t *areas) {
  struct async_private_data data;
  snd_async_handler_t *ahandler;
  int err;
  
  data.samples = samples;
  data.areas = areas;
  data.phase = 0;
  if((err = snd_async_add_pcm_handler(&ahandler, handle, transfer_callback, &data)) < 0) {
    printf("Unable to register async handler\n");
    exit(EXIT_FAILURE);
  }
  for(int count = 0; count < 2; count++) {
    generate_sine(areas, 0, period_size, &data.phase);
    err = snd_pcm_writei(handle, samples, period_size);
    if(err < 0) {
      printf("Initial write error: %s\n", snd_strerror(err));
      exit(EXIT_FAILURE);
    }
    if(err != period_size) {
      printf("Initial write error: written %i expected %li\n", err, period_size);
      exit(EXIT_FAILURE);
    }
  }
  if(snd_pcm_state(handle) == SND_PCM_STATE_PREPARED) {
    if((err = snd_pcm_start(handle)) < 0) {
      printf("Start error: %s\n", snd_strerror(err));
      exit(EXIT_FAILURE);
    }
  }
    /* because all other work is done in the signal handler,
       suspend the process */
  while (1) {
#if DEBUG
    printf(" *** State dump ***\n");
    if((err = snd_pcm_status_dump(status, output)) < 0) {
      printf("Cannot dump PCM status information: %s\n", snd_strerror(err));
      exit(EXIT_FAILURE);
    }
#endif
    sleep(1); //! no llega a completarse porque el proceso es interrumpido debido al callback
  }
  return 0;
}

static int set_hwparams(snd_pcm_t *handle,
			snd_pcm_hw_params_t *params) {
  unsigned int rrate;
  snd_pcm_uframes_t size;
  int err, dir;
  /* choose all parameters */
  if((err = snd_pcm_hw_params_any(handle, params)) < 0) {
    printf("Broken configuration for playback: no configurations available: %s\n", snd_strerror(err));
    return err;
  }
  /* set the interleaved read/write format */
  if((err = snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
    printf("Access type not available for playback: %s\n", snd_strerror(err));
    return err;
  }
  /* set the sample format */
  if((err = snd_pcm_hw_params_set_format(handle, params, format)) < 0) {
    printf("Sample format not available for playback: %s\n", snd_strerror(err));
    return err;
  }
  /* set the count of channels */
  if((err = snd_pcm_hw_params_set_channels(handle, params, channels)) < 0) {
    printf("Channels count (%u) not available for playbacks: %s\n", channels, snd_strerror(err));
    return err;
  }
    /* set the stream rate */
  rrate = rate;
  if((err = snd_pcm_hw_params_set_rate_near(handle, params, &rrate, 0)) < 0) {
    printf("Rate %uHz not available for playback: %s\n", rate, snd_strerror(err));
    return err;
  }
  if(rrate != rate) {
    printf("Rate doesn't match (requested %uHz, get %iHz)\n", rate, err);
    return -EINVAL;
  }
  /* set the buffer time */
  if((err = snd_pcm_hw_params_set_buffer_time_near(handle, params, &buffer_time, &dir)) < 0) {
    printf("Unable to set buffer time %u for playback: %s\n", buffer_time, snd_strerror(err));
    return err;
  }
  if((err = snd_pcm_hw_params_get_buffer_size(params, &size)) < 0) {
    printf("Unable to get buffer size for playback: %s\n", snd_strerror(err));
    return err;
  }
  buffer_size = size;
  /* set the period time */
  if((err = snd_pcm_hw_params_set_period_time_near(handle, params, &period_time, &dir)) < 0) {
    printf("Unable to set period time %u for playback: %s\n", period_time, snd_strerror(err));
    return err;
  }
  if((err = snd_pcm_hw_params_get_period_size(params, &size, &dir)) < 0) {
    printf("Unable to get period size for playback: %s\n", snd_strerror(err));
    return err;
  }
  period_size = size;
  /* write the parameters to device */
  if((err = snd_pcm_hw_params(handle, params)) < 0) {
    printf("Unable to set hw params for playback: %s\n", snd_strerror(err));
    return err;
  }
  return 0;
}
static int set_swparams(snd_pcm_t *handle, snd_pcm_sw_params_t *swparams) {
  int err;
  /* get the current swparams */
  if((err = snd_pcm_sw_params_current(handle, swparams)) < 0) {
    printf("Unable to determine current swparams for playback: %s\n", snd_strerror(err));
    return err;
  }
  /* start the transfer when the buffer is almost full: */
  /* (buffer_size / avail_min) * avail_min */
  if((err = snd_pcm_sw_params_set_start_threshold(handle, swparams, buffer_size - period_size)) < 0) {
    printf("Unable to set start threshold mode for playback: %s\n", snd_strerror(err));
    return err;
  }
  if((err = snd_pcm_sw_params_set_avail_min(handle, swparams, period_size)) < 0) {
    printf("Unable to set avail min for playback: %s\n", snd_strerror(err));
    return err;
  }
  if((err = snd_pcm_sw_params_set_period_event(handle, swparams, 1)) < 0) {
    printf("Unable to set period event: %s\n", snd_strerror(err));
    return err;
  }
  /* write the parameters to the playback device */
  if((err = snd_pcm_sw_params(handle, swparams)) < 0) {
    printf("Unable to set sw params for playback: %s\n", snd_strerror(err));
    return err;
  }
  return 0;
}

int main(int argc, char *argv[]) {
  snd_pcm_t *handle;
  int err;
  snd_pcm_hw_params_t *hwparams;
  snd_pcm_sw_params_t *swparams;

  if((err = snd_output_stdio_attach(&output, stdout, 0)) < 0) {
    printf("Output failed: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }
  
  if((err = snd_pcm_open(&handle, device, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
    printf("Playback open error: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }

  snd_pcm_status_alloca(&status);
  if((err = snd_pcm_status(handle, status)) < 0) {
    printf("Cannot get pcm status: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }

  snd_pcm_hw_params_alloca(&hwparams);
  snd_pcm_sw_params_alloca(&swparams);

  if((err = set_hwparams(handle, hwparams)) < 0) {
    printf("Setting of hwparams failed: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }
  if((err = set_swparams(handle, swparams)) < 0) {
    printf("Setting of swparams failed: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }
  
  signed short *samples = malloc((period_size * channels * snd_pcm_format_physical_width(format)) / 8);
  if(samples == NULL) {
    printf("No enough memory\n");
    exit(EXIT_FAILURE);
  }

  // Set up channel areas
  snd_pcm_channel_area_t *areas = calloc(channels, sizeof(snd_pcm_channel_area_t));
  if(areas == NULL) {
    printf("No enough memory\n");
    exit(EXIT_FAILURE);
  }
  for(int chn = 0; chn < channels; chn++) {
    areas[chn].addr = samples;
    areas[chn].first = chn * snd_pcm_format_physical_width(format);
    areas[chn].step = channels * snd_pcm_format_physical_width(format);
  }

  if((err = snd_pcm_prepare(handle)) < 0) {
    printf("Can't prepare device: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }
  if((err = snd_pcm_dump(handle, output)) < 0) {
    printf("Unable to dump hw setup information: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }
  transfer_loop(handle, samples, areas);
  
  free(areas);
  free(samples);
  snd_pcm_close(handle);
  return 0;
}
