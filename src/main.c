#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <assert.h>
#include <complex.h>
#include <string.h>
#include <liquid/liquid.h>

#ifdef __APPLE__
  #include <iio/iio.h>
#else
  #include <iio.h>
#endif

enum iodev { RX, TX };

#define MHZ(x) ((long long)( x * 1e6 + 0.5f ))
#define GHZ(x) ((long long)( x * 1e9 + 0.5f ))

FILE * fp;
static int running = true;

void * transmit_thread( void * );
int transmit(struct iio_context *);
void * receive_thread( void * );
int receive(struct iio_context *);
void printDeviceChannels(struct iio_device *);
void install_sigint_handler(void);
static void sigint_handler(int);

static int callback(
  unsigned char *  _header,
  int              _header_valid,
  unsigned char *  _payload,
  unsigned int     _payload_len,
  int              _payload_valid,
  framesyncstats_s _stats,
  void *           _userdata
)
{
  printf("callback invoked\n");
  printf("header valid? %d\n", _header_valid);
  printf("payload valid? %d\n", _payload_valid);
  if( _payload_valid )
    printf("%s\n", _payload);

  framesyncstats_print(&_stats);
  return 0;
}

int main(int argc, char ** argv)
{
  struct iio_context * ctx;
  struct iio_device * phy;
  pthread_t tx_thread, rx_thread;

  install_sigint_handler();

  fp = fopen("output.txt", "w");

  ctx = iio_create_context_from_uri("ip:192.168.2.1");

  assert(ctx != NULL && "IIO context could not be created");

  pthread_create( &tx_thread, NULL, transmit_thread, (void *)ctx );
  pthread_create( &rx_thread, NULL, receive_thread,  (void *)ctx );

  pthread_join( tx_thread, NULL );
  pthread_join( rx_thread, NULL );

  iio_context_destroy(ctx);

  fclose(fp);

  return 0;
}

void * transmit_thread( void * arg )
{
  transmit( (struct iio_context *)arg );
  return NULL;
}

int transmit(struct iio_context * ctx)
{
  int status = 0;
  struct iio_device  * dev   = NULL;
  struct iio_device  * phy   = NULL;
  struct iio_channel * chn   = NULL;
  struct iio_channel * tx0_i = NULL;
  struct iio_channel * tx0_q = NULL;
  struct iio_buffer  * txbuf = NULL;

  // find transmit device
  dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");

  phy = iio_context_find_device(ctx, "ad9361-phy");

  chn = iio_device_find_channel(phy, "voltage0", true);

  if( iio_channel_attr_write(chn, "rf_port_select", "A") < 0 )
  {
    printf("Failed to select TX RF port\n");
    status = -1;
    goto cleanup;
  }

  if( iio_channel_attr_write_longlong(chn, "rf_bandwidth", MHZ(1.5)) < 0 )
  {
    printf("Failed to set RF bandwidth\n");
    status = -1;
    goto cleanup;
  }

  if( iio_channel_attr_write_longlong(chn, "sampling_frequency", MHZ(2.5)) < 0 )
  {
    printf("Failed to write sampling frequency\n");
    status = -1;
    goto cleanup;
  }

  chn = iio_device_find_channel(phy, "altvoltage1", true);

  if( iio_channel_attr_write_longlong(chn, "frequency", GHZ(2.4)) < 0 )
  {
    printf("Failed to write TX LO frequency\n");
    status = -1;
    goto cleanup;
  };

  tx0_i = iio_device_find_channel(dev, "voltage0", true);
  tx0_q = iio_device_find_channel(dev, "voltage1", true);

  if( tx0_i == NULL || tx0_q == NULL )
  {
    printf("Failed to acquire TX i/q channels\n");
    status = -1;
    goto cleanup;
  }

  iio_channel_enable(tx0_i);
  iio_channel_enable(tx0_q);

  txbuf = iio_device_create_buffer(dev, 4096, false);

  if( !txbuf )
  {
    perror("Could not create TX buffer");
    status = -1;
    goto cleanup;
  }

  framegen64 fg = framegen64_create();
  unsigned char header[8];
  unsigned char payload[64];
  complex float frame[LIQUID_FRAME64_LEN];

  strncpy(payload, "Hello World!", 64);

  framegen64_execute(fg, header, payload, frame);


  while( running )
  {
    ssize_t nbytes_tx;
    char * p_dat, * p_end;
    ptrdiff_t p_inc;

    nbytes_tx = iio_buffer_push(txbuf);

    if( nbytes_tx < 0 )
    {
      printf("Error pushing tx buffer\n");
      running = false;
    }
    else
    {
      p_inc = iio_buffer_step(txbuf);
      p_end = iio_buffer_end(txbuf);
      p_dat = iio_buffer_first(txbuf, tx0_i);

      for (int i=0; p_dat < p_end; p_dat += p_inc, ++i )
      {
        // int b0, b1;
        //
        // if( (i/256) % 4 == 0 ) { b0 = 1; b1 = 1; }
        // if( (i/256) % 4 == 1 ) { b0 =-1; b1 = 1; }
        // if( (i/256) % 4 == 2 ) { b0 =-1; b1 =-1; }
        // if( (i/256) % 4 == 3 ) { b0 = 1; b1 =-1; }

        // ((int16_t *)p_dat)[0] = ((int16_t)( 2047 * cos( 2 * M_PI * i / 4096 ) )) << 4;
        // ((int16_t *)p_dat)[1] = ((int16_t)( 2047 * sin( 2 * M_PI * i / 4096 ) )) << 4;

        // ((int16_t *)p_dat)[0] = ((int16_t)( 2047 * b0 )) << 4;
        // ((int16_t *)p_dat)[1] = ((int16_t)( 2047 * b1 )) << 4;

        if(i < LIQUID_FRAME64_LEN)
        {
          ((int16_t *)p_dat)[0] = ((int16_t)( 100 * crealf( frame[i]) )) << 4;
          ((int16_t *)p_dat)[1] = ((int16_t)( 100 * cimagf( frame[i]) )) << 4;
        }
        else
        {
          ((int16_t *)p_dat)[0] = ((int16_t)( 0 )) << 4;
          ((int16_t *)p_dat)[1] = ((int16_t)( 0 )) << 4;
        }
      }
    }

  }

cleanup:
  running = false;
  if(txbuf) iio_buffer_destroy(txbuf);
  if(tx0_i) iio_channel_disable(tx0_i);
  if(tx0_q) iio_channel_disable(tx0_q);

  framegen64_destroy(fg);

  return status;
}

void * receive_thread( void * arg )
{
  receive( (struct iio_context *)arg );
  return NULL;
}

int receive(struct iio_context * ctx)
{
  int status = 0;
  struct iio_device  * dev   = NULL;
  struct iio_device  * phy   = NULL;
  struct iio_channel * chn   = NULL;
  struct iio_channel * rx0_i = NULL;
  struct iio_channel * rx0_q = NULL;
  struct iio_buffer  * rxbuf = NULL;

  dev = iio_context_find_device(ctx, "cf-ad9361-lpc");

  phy = iio_context_find_device(ctx, "ad9361-phy");

  chn = iio_device_find_channel(phy, "voltage0", false);

  if( iio_channel_attr_write(chn, "rf_port_select", "A_BALANCED") < 0 )
  {
    printf("Failed to select RX RF port\n");
    status = -1;
    goto cleanup;
  }

  if( iio_channel_attr_write_longlong(chn, "rf_bandwidth", MHZ(2)) < 0 )
  {
    printf("Failed to write RX RF bandwdith\n");
    status = -1;
    goto cleanup;
  }

  if( iio_channel_attr_write_longlong(chn, "sampling_frequency", MHZ(2.5)) < 0 )
  {
    printf("Failed to write RX sampling frequency\n");
    status = -1;
    goto cleanup;
  }

  chn = iio_device_find_channel(phy, "altvoltage0", true);

  if( iio_channel_attr_write_longlong(chn, "frequency", GHZ(2.3999)) < 0 )
  {
    printf("Failed to write RX LO frequency\n");
    status = -1;
    goto cleanup;
  }

  rx0_i = iio_device_find_channel(dev, "voltage0", 0);
  rx0_q = iio_device_find_channel(dev, "voltage1", 0);

  iio_channel_enable(rx0_i);
  iio_channel_enable(rx0_q);

  rxbuf = iio_device_create_buffer(dev, 4096, false);

  if( !rxbuf )
  {
    perror("Could not create RX buffer");
    running = false;
  }

  framesync64 fs = framesync64_create(callback, NULL);

  while( running )
  {
    void * p_dat, * p_end, * t_dat;
    ptrdiff_t p_inc;

    // carrier tracking
    float phase = 0;
    float error[2] = {0};
    float freq = 0;
    complex float demod;
    float i_sign = 0;
    float q_sign = 0;
    float ka = 0.05;
    float kp = 0.002;
    float b0 = (ka + kp);
    float b1 = -kp;


    iio_buffer_refill(rxbuf);

    p_inc = iio_buffer_step(rxbuf);
    p_end = iio_buffer_end(rxbuf);

    for(p_dat = iio_buffer_first(rxbuf, rx0_i); p_dat < p_end; p_dat += p_inc, t_dat += p_inc)
    {
      const int16_t i = ((int16_t *)p_dat)[0];
      const int16_t q = ((int16_t *)p_dat)[1];
      complex float cpx = ((float)i) + I * ((float)q);
      framesync64_execute(fs, &cpx, 1);
    }
  }

cleanup:
  running = false;

  if( rxbuf ) iio_buffer_destroy(rxbuf);
  if( rx0_i ) iio_channel_disable(rx0_i);
  if( rx0_q ) iio_channel_disable(rx0_q);

  framesync64_destroy(fs);

  return 0;
}

void printDeviceChannels(struct iio_device* dev)
{
	int num_channels = iio_device_get_channels_count(dev);
	int j = 0;
	for (j=0; j<num_channels; j++)
  {
		struct iio_channel* chn;
		chn =  iio_device_get_channel(dev, j);
		int num_attrs = iio_channel_get_attrs_count(chn);
		printf("\n    Channel %i, Attributes %i\n", j, num_attrs);
		const char* id = iio_channel_get_id(chn);
		const char* name = iio_channel_get_name(chn);
		bool isoutput = iio_channel_is_output(chn);
		bool isscanel = iio_channel_is_scan_element(chn);

		printf("      %-30s: %s\n", "id", id);
		printf("      %-30s: %s\n", "name", name);
		if (isoutput) { printf("      %-30s: %s\n", "I/O", "output"); }
		else { printf("      %-30s: %s\n", "I/O", "input"); }
		printf("      %-30s: %i\n", "scan_element", isscanel);

		// Get all attributes for this channel
		int k = 0;
		for (k=0; k<num_attrs; k++)
    {
			const char* attr = iio_channel_get_attr(chn, k);

			char buf[2048];
			int ret = iio_channel_attr_read(chn, attr, buf, sizeof(buf));

      if (ret > 0)
      {
				printf("      %-30s: %s\n", attr, buf);
			}
      else
      {
				iio_strerror(-ret, buf, sizeof(buf));
				fprintf(stderr, "[ERROR]  Unable to read attribute %s: %s\n", attr, buf);
			}

		}
	}
}

static void sigint_handler(int signum)
{
  printf("Caught Ctrl-C, exiting now.\n");
  running = false;
}

void install_sigint_handler(void)
{
  struct sigaction sa;

  sa.sa_handler = sigint_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESTART;

  if( sigaction( SIGINT, &sa, NULL ) == -1 )
  {
    perror("sigaction");
    exit(1);
  }

}
