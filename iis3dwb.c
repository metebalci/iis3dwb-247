#define _GNU_SOURCE
#define _FILE_OFFSET_BITS 64

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

// modify these if there is a need
#define SPI_DEVICE  "/dev/spidev0.0"
#define FULL_SCALE  2
// do not modify these unless absolutely necessary
#define SPI_SPEED   10000000
#define SPI_MODE    SPI_MODE_0
#define TEMP_FILE       "fifo.bin"
#define TEMP_FILE_META  "fifo.bin.meta"

// only to be changed if errors are encountered
// number of lines, limited by available memory
#define NUM_FIFO_RX_LINES (1024*1024)
// warning this should be maximum 254
#define FIFO_RX_LINE_MAX_SAMPLES 128

// uncomment this to print out SPI communications
//#define PRINT_SPI_TRACE

// fd of SPI device
int fd = 0;
int ret = 0;
// breaks are used to early terminate the loops in threads
bool sensor_reader_break = false;
bool file_writer_break = false;
// flag to check if file writer is terminated or still running 
bool file_writer_terminated = false;
// the duration to collect data is given at command line
uint32_t duration = 0;
// uint32_t finishes around 22h
// to record >22h, uint64_t has to be used for nsamples
// but the bigger problem is the file size
// fifo.bin becomes >32-bit after around 6h
// so 64-bit file operations has to be used 
uint64_t nsamples = 0;
// for spi transfers
struct spi_ioc_transfer trx = {0};
// to sync threads for accessing shared data
pthread_mutex_t lock;

// buffers for spi transfers used for register access only
uint8_t tx_buffer[4096];
uint8_t rx_buffer[4096];

// this is only used for dumping the register values for debug
// used by dump_regs()
const char *all_regs[] = {
  "02 PIN_CTRL",
  "07 FIFO_CTRL1",
  "08 FIFO_CTRL2",
  "09 FIFO_CTRL3",
  "0A FIFO_CTRL4",
  "0B COUNTER_BDR_REG1",
  "0C COUNTER_BDR_REG2",
  "0D INT1_CTRL",
  "0E INT2_CTRL",
  "0F WHO_AM_I",
  "10 CTRL1_XL",
  "12 CTRL3_C",
  "13 CTRL4_C",
  "14 CTRL5_C",
  "15 CTRL6_C",
  "16 CTRL7_C",
  "17 CTRL8_XL",
  "19 CTRL10_C",
  "1A ALL_INT_SRC",
  "1B WAKE_UP_SRC",
  "1E STATUS",
  "56 SLOPE_EN",
  "58 INTERRUPTS_EN",
  "5B WAKE_UP_THS",
  "5C WAKE_UP_DUR",
  "5E MD1_CFG",
  "5F MD2_CFG",
  "63 INTERNAL_FREQ_FINE",
  "73 X_OFS_USR",
  "74 Y_OFS_USR",
  "75 Z_OFS_USR",
  NULL
};

// these are the registers used in this code
#define REG_FIFO_CTRL3  0x09
#define REG_FIFO_CTRL4  0x0A
#define REG_WHO_AM_I    0x0F
#define REG_CTRL1_XL    0x10
#define REG_CTRL3_C     0x12
#define REG_CTRL5_C     0x14
#define REG_CTRL6_C     0x15
#define REG_CTRL10_C    0x19
#define REG_FIFO_STATUS1  0x3A
#define REG_FIFO_STATUS2  0x3B
#define REG_TIMESTAMP2  0x42
#define REG_INTERNAL_FREQ_FINE  0x63
#define REG_FIFO_DATA_OUT_TAG 0x78

// [0] is nsamples in line, [1] is dummy
#define FIFO_RX_LINE_SIZE (2+(FIFO_RX_LINE_MAX_SAMPLES*7))
// tx is used only for sending read reg command
// all bytes other than the first is dummy
uint8_t fifo_tx[FIFO_RX_LINE_SIZE] = {0};
// this is like a circular buffer of length NUM_FIFO_RX_LINES
// but instead of single byte, it keeps FIFO_RX_LINE_SIZE of bytes
// like circular buffer of arrays
// not all bytes are utilized, the number of samples (bytes/7) in a line
// is stored as the first byte of the line
// second byte of the line is a dummy (due to SPI transactions)
uint8_t fifo_rx[NUM_FIFO_RX_LINES * FIFO_RX_LINE_SIZE] = {0};
// points to next available slot
uint32_t fifo_rx_write_idx = 0;
// points to next read slot but when fifo_rx_nlines > 0
// otherwise read_idx and write_idx points to same slot
uint32_t fifo_rx_read_idx = 0;
// number of slots utilized, maximum is NUM_FIFO_RX_LINES
uint32_t fifo_rx_nlines = 0;
// histogram like data of fifo_rx usage
// index is nsample (0..FIFO_RX_LINE_MAX_SAMPLES)
// value is how many times this nsample is used for a line
uint32_t line_usage_histogram[FIFO_RX_LINE_MAX_SAMPLES] = {0};

// dump 
void hexdump(uint8_t* p, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    if (i == 0) {
      printf(" [%02X]", p[i]);
    } else {
      printf(" %02X", p[i]);
    }
  }
}

void check_ret(const char* msg) {
  if (ret != 0) {
    printf(msg);
    close(fd);
    exit(EXIT_FAILURE);
  }
}

void quit(int code) {
  if (fd >= 0) {
    close(fd);
  }
  exit(code);
}

void quits() {
  quit(EXIT_SUCCESS);
}

void quitf() {
  quit(EXIT_FAILURE);
}

void spi_configure() {

  fd = open(SPI_DEVICE, O_RDWR);
  if (fd < 0) {
    printf("cannot open %s\n", SPI_DEVICE);
    exit(EXIT_FAILURE);
  }

  uint32_t v;

  v = SPI_MODE;
  ret = ioctl(fd, SPI_IOC_WR_MODE32, &v);
  check_ret("cannot write SPI MODE32");

  v = 0;
  ret = ioctl(fd, SPI_IOC_WR_LSB_FIRST, &v);
  check_ret("cannot write SPI LSB_FIRST");

  v = 8;
  ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &v);
  check_ret("cannot write SPI BITS_PER_WORD");

  v = SPI_SPEED;
  ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &v);
  check_ret("cannot write SPI MAX_SPEED_HZ");

  trx.bits_per_word = 8;
  // use the device speed
  trx.speed_hz = 0;
  trx.delay_usecs = 0;
  // use cs between xfers
  // if set, cs is kept active during multiple xfers
  // no idea when then it is made not active
  trx.cs_change = 0;

}

// the core SPI xfer function
void spi_xfer(uint8_t* spi_tx_buffer, uint8_t* spi_rx_buffer, uint32_t len) {

  assert(len < 4096);

  trx.tx_buf = (unsigned long) spi_tx_buffer;
  trx.rx_buf = (unsigned long) spi_rx_buffer;
  trx.len = len;

#ifdef PRINT_SPI_TRACE
    printf(">");
    hexdump(spi_tx_buffer, len);
    printf("\n");
#endif

  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &trx);

  if (ret < 0) {
    printf("SPI xfer failed: %d, %s\n", errno, strerror(errno));
    quitf();
  }

#ifdef PRINT_SPI_TRACE
    printf("<");
    hexdump(spi_rx_buffer, len);
    printf("\n");
#endif

}

// writes tx_buffer[0:len] to reg
// rx_buffer[0:len] will have dummy values
void spi_write(uint8_t reg, uint32_t len) {

  assert(reg < 0x80);
  assert(len < 4096);

#ifdef PRINT_SPI_TRACE
    printf("SPI.write %02Xh -----\n", reg);
#endif

  // write when bit 0=0
  tx_buffer[0] = 0x7F & reg;

  spi_xfer(tx_buffer, rx_buffer, len);

#ifdef PRINT_SPI_TRACE
    printf("-----\n");
#endif

}

// reads reg to rx_buffer[0:len]
// tx_buffer[1:len] is considered dummy values, it is zeroed
void spi_read(uint8_t reg, uint32_t len) {

  assert(reg < 0x80);
  assert(len < 4096);

#ifdef PRINT_SPI_TRACE
    printf("SPI.read %02Xh ------\n", reg);
#endif

  memset(tx_buffer, 0, len);
  // read when bit 0=1
  tx_buffer[0] = 0x80 | reg;

  spi_xfer(tx_buffer, rx_buffer, len);

#ifdef PRINT_SPI_TRACE
    printf("-----\n");
#endif

}

// writes single byte val to reg
void spi_write_reg(uint8_t reg, uint8_t val) {

  assert(reg < 0x80);

  tx_buffer[1] = val;

  spi_write(reg, 2);

}

// reads single byte val from reg
uint8_t spi_read_reg(uint8_t reg) {

  assert(reg < 0x80);

  tx_buffer[1] = 0;
  spi_read(reg, 2);

  return rx_buffer[1];

}

// modifes reg
// reads reg to temp value
// creates a mask and masks the temp value
// creates a shifted value and ors with the masked temp value
// writes the ored masked temp value back to reg
void spi_modify_reg(uint8_t reg, uint8_t val, uint8_t bitpos, uint8_t bitlen) {

  assert(reg < 0x80);
  assert(bitpos < 8);
  assert(bitlen <= 8);

  //printf("modify val=%d bitpos=%d bitlen=%d\n", val, bitpos, bitlen);

  uint8_t current = spi_read_reg(reg);
  uint8_t mask = 0;
  for (uint8_t i = 0; i < bitlen; i++) {
    mask = (mask << 1) | 1;
  }
  mask = mask << bitpos;
  //printf("current=0x%02X mask=0x%02X\n", current, mask);
  current = current & ~mask;
  //printf("masked current=0x%02X\n", current);
  current = current | (val << bitpos);
  //printf("new current=0x%02X\n", current);
  spi_write_reg(reg, current);

}

// prints out regs
// for debug purposes
void dump_regs() {

  printf("\n------ REGISTERS ------\n");

  uint32_t i = 0;

  while (all_regs[i] != NULL) {

    const char *reg = all_regs[i++];
    char addrs[3] = {0};
    addrs[0] = reg[0];
    addrs[1] = reg[1];
    uint32_t addr = strtoul(addrs, NULL, 16);

    uint8_t val = spi_read_reg(addr);

    printf("%-20s [%02Xh]: 0x%02X\n", &reg[3], addr, val);

  }

  printf("------\n\n");

}

uint8_t whoami() {
  return spi_read_reg(REG_WHO_AM_I);
}

void iis3dwb_swreset() {

  spi_modify_reg(REG_CTRL3_C, 1, 0, 1);
  while ((spi_read_reg(REG_CTRL3_C) & 0x01) != 0);

}

void iis3dwb_set_fullscale(uint8_t g) {

  uint8_t fs = 0;

  switch (g) {
    case 2:
      fs = 0b00;
      break;
    case 4:
      fs = 0b10;
      break;
    case 8:
      fs = 0b11;
      break;
    case 16:
      fs = 0b01;
      break;
    default:
      printf("ERROR. invalid full scale value");
      quitf();
  }

  spi_modify_reg(REG_CTRL1_XL, fs, 2, 2);

}

void iis3dwb_configure() {

  // enable block data update
  spi_modify_reg(REG_CTRL3_C, 1, 6, 1);
  // enable automatic register address increment
  spi_modify_reg(REG_CTRL3_C, 1, 2, 1);
  // enable wraparound
  spi_modify_reg(REG_CTRL5_C, 0b01, 5, 2);
  // enable timestamp counter
  spi_write_reg(REG_CTRL10_C, 0x20);
  // batch accelerometer data
  spi_write_reg(REG_FIFO_CTRL3, 0x0A);
  // batch timestamps, decimation 1
  spi_modify_reg(REG_FIFO_CTRL4, 0b01, 6, 2);
  // select fifo mode, 110=continuous
  spi_modify_reg(REG_FIFO_CTRL4, 0b110, 0, 3);
  // reset timestamp counter 
  spi_write_reg(REG_TIMESTAMP2, 0xAA);

}

void iis3dwb_enable() {
  spi_modify_reg(REG_CTRL1_XL, 0b101, 5, 3);
}

int8_t iis3dwb_internal_freq_fine() {
  return (int8_t) spi_read_reg(REG_INTERNAL_FREQ_FINE);
}

// the formula is given in the datasheet
double iis3dwb_odr_actual() {
  const double odr = 26667.0;
  const int8_t iff = iis3dwb_internal_freq_fine();
  return odr + (0.0015 * iff * odr);
}

// the formula is given in the datasheet
double iis3dwb_timestamp_resolution_actual() {
  const int8_t iff = iis3dwb_internal_freq_fine();
  return 1.0 / (80000.0 + (0.0015 * iff * 80000.0));
}

void sig_handler(int signum) {
  sensor_reader_break = true;
  file_writer_break = true;
}

// the file_writer thread
// reads from fifo_rx lines and writes the data to output file
void *file_writer(void *vargp) {

  FILE* fp = fopen(TEMP_FILE, "wb");

  uint64_t samples_written = 0;
  uint32_t cnt_read_idx_reset = 0;

  // samples_written can be = or > than nsamples
  while (samples_written < nsamples) {

    if (file_writer_break) break;

    // wait until a new line appears
    // there is no mutex here because it is not modified
    // the worst case is it might be read as 0, 
    //  and in the next iteration it will be >0
    if (fifo_rx_nlines > 0) {

      uint8_t* current_line = fifo_rx + fifo_rx_read_idx*FIFO_RX_LINE_SIZE;

      // how many samples in this line
      uint8_t current_line_nsamples = current_line[0];

      // [0] is nsamples in line=len
      // [1] is dummy
      // actual data is [2:len*7]
      fwrite(current_line+2, 
          sizeof(uint8_t), 
          current_line_nsamples*7, 
          fp);

      samples_written += current_line_nsamples;

      // increment read_idx to point next location
      // this location might not contain valid data yet
      // this is checked in the next iteration with fifo_rx_nlines>0
      fifo_rx_read_idx++;
      if (fifo_rx_read_idx == NUM_FIFO_RX_LINES) {
        cnt_read_idx_reset++;
        fifo_rx_read_idx = 0;
      }

      // only fifo_rx_nlines is shared between threads
      // hence the mutex
      pthread_mutex_lock(&lock);
      fifo_rx_nlines--;
      pthread_mutex_unlock(&lock);

    }

  }

  fclose(fp);

  pthread_mutex_lock(&lock);
  printf("\n");
  printf("------ file_writer stats ------\n");
  printf("samples_written:        %llu\n", samples_written);
  printf("bytes_written:          %llu\n", (samples_written*7));
  printf("fifo_rx_read_idx:       %u\n", fifo_rx_read_idx);
  printf("cnt_read_idx_reset:     %u\n", cnt_read_idx_reset);
  printf("------\n\n");
  // mark terminated, so sensor_reader can also terminate without waiting
  file_writer_terminated = true;
  pthread_mutex_unlock(&lock);

  return NULL;

}

// the sensor_reader thread
// reads from sensor into fifo_rx lines
void *sensor_reader(void *vargp) {

  // used for reading from FIFO
  // addr incremented automatically and wraparound is enabled
  fifo_tx[0] = 0x80 | REG_FIFO_DATA_OUT_TAG;

  uint64_t samples_read = 0;
  uint32_t cnt_ovrs = 0;
  uint32_t last_cnt_ovrs = 0;
  uint32_t max_diff_fifo = 0;
  uint64_t total_diff_fifo = 0;
  uint64_t cnt_pos_diff_fifos = 0;
  uint64_t cnt_neg_diff_fifos = 0;
  uint32_t cnt_line_is_too_small = 0;
  uint32_t last_cnt_line_is_too_small = 0;
  uint32_t cnt_write_idx_reset = 0;
  uint32_t max_fifo_rx_nlines = 0;

  uint8_t samples_to_read = 0;

  struct timespec tstart={0,0}, tlast={0,0}, tnow={0,0};

  // disable printf buffering for progress updates
  setbuf(stdout, NULL);

  clock_gettime(CLOCK_MONOTONIC, &tstart);

  // enable the sensor 
  iis3dwb_enable();

  // samples_read can be = or > than nsamples
  while (samples_read < nsamples) {

    if (sensor_reader_break) break;

    spi_read(REG_FIFO_STATUS1, 3);
    const uint8_t fifo_status1 = rx_buffer[1];
    const uint8_t fifo_status2 = rx_buffer[2];
    const uint16_t diff_fifo = ((fifo_status2 & 0x03) << 8) | fifo_status1;
    total_diff_fifo += diff_fifo;

    const bool ovr = (rx_buffer[2] & 0x40) != 0;

    if (ovr) {

      cnt_ovrs++;

      // when overrun happens
      // discard until a timestamp is found
      // this implementation is not totally correct and not optimized
      // it reads one sample at a time,
      //  assuming the next timestamp is very close
      // also the timestamp is not kept,
      //  thus the next accelerometer data will also be skipped when saving 
      while (true) {
        spi_xfer(fifo_tx, rx_buffer, 8);
        if ((rx_buffer[1] >> 3) == 0x04) {
          break;
        }
      }

    } else if (diff_fifo > 0) {

      cnt_pos_diff_fifos++;
      if (diff_fifo > max_diff_fifo) max_diff_fifo = diff_fifo;

      // check if fifo_rx is full
      // when it is full, it is a fatal condition
      // only fifo_rx_nlines is shared between threads
      // hence the mutex
      pthread_mutex_lock(&lock);
      const bool fifo_rx_full = (fifo_rx_nlines == NUM_FIFO_RX_LINES);
      if (fifo_rx_nlines > max_fifo_rx_nlines) {
        max_fifo_rx_nlines = fifo_rx_nlines;
      }
      pthread_mutex_unlock(&lock);

      // sensor read does not wait for fifo_rx to have space
      // since sensor is producing data continuously
      // if this happens, unfortunately it has to fail
      if (fifo_rx_full) {
        printf("\nERROR. no fifo_rx lines left\n");
        sensor_reader_break = true;
        break;
      }

      // not all diff_fifo is read
      // max FIFO_RX_LINE_MAX_SAMPLES is read from sensor's FIFO
      if (diff_fifo > FIFO_RX_LINE_MAX_SAMPLES) {
        cnt_line_is_too_small++;
        samples_to_read = FIFO_RX_LINE_MAX_SAMPLES;
      } else {
        samples_to_read = diff_fifo;
      } 

      // every second, display progress as a single char
      // . means everything is ok
      // [0..F] means this much line is too small event happened in last second
      // X means >Fh times line is too small event happened
      clock_gettime(CLOCK_MONOTONIC, &tnow);
      if (((tnow.tv_sec - tlast.tv_sec) + 
            1.0e-9 * (tnow.tv_nsec - tlast.tv_nsec)) > 1.0) {
        const uint32_t temp1 = cnt_ovrs - last_cnt_ovrs;
        const uint32_t temp2 = cnt_line_is_too_small - last_cnt_line_is_too_small;
        if (temp1 > 0) {
          printf("!");
        } else if (temp2 == 0) {
          printf(".");
        } else if (temp2 < 10) {
          // 48 is ascii 0, if temp is 1, =49=ascii for 1
          printf("%c", (temp2 + 48));
        } else if (temp2 < 16) {
          // if temp is 10, + 55 is 65 which is ascii for A
          printf("%c", (temp2 + 55 ));
        } else {
          printf("X");
        }
        last_cnt_ovrs = cnt_ovrs;
        last_cnt_line_is_too_small = cnt_line_is_too_small;
        clock_gettime(CLOCK_MONOTONIC, &tlast);
      }

      uint8_t* current_line = fifo_rx + fifo_rx_write_idx*FIFO_RX_LINE_SIZE;

      spi_xfer(fifo_tx, current_line+1, samples_to_read*7+1);

      current_line[0] = samples_to_read;
      line_usage_histogram[samples_to_read]++;
      samples_read += samples_to_read; 

      // increment write_idx to point next location
      // this location might not be free
      // but this is checked on the next iteration
      fifo_rx_write_idx++;
      if (fifo_rx_write_idx == NUM_FIFO_RX_LINES) {
        cnt_write_idx_reset++;
        fifo_rx_write_idx = 0;
      }

      // only fifo_rx_nlines is shared between threads
      // hence the mutex
      pthread_mutex_lock(&lock);
      fifo_rx_nlines++;
      pthread_mutex_unlock(&lock);

    } else {

      cnt_neg_diff_fifos++;

    }

  }

  pthread_mutex_lock(&lock);
  printf("\n\n");

  // print status if no error happened
  printf("------ sensor_reader stats ------\n");
  printf("samples_read:           %llu\n", samples_read);
  printf("cnt_ovrs:               %u\n", cnt_ovrs);
  printf("max_diff_fifo:          %u\n", max_diff_fifo);
  printf("avg_diff_fifo:          %.1lf\n", 
      ((double)total_diff_fifo/(double)cnt_pos_diff_fifos));
  printf("cnt_pos_diff_fifos:     %llu\n", cnt_pos_diff_fifos);
  printf("cnt_neg_diff_fifos:     %llu\n", cnt_neg_diff_fifos);
  printf("fifo_rx_write_idx:      %u\n", fifo_rx_write_idx);
  printf("cnt_write_idx_reset:    %u\n", cnt_write_idx_reset);
  printf("------\n\n");

  printf("------ fifo rx stats ------\n");
  printf("max_fifo_rx_nlines:     %u\n", cnt_write_idx_reset);
  printf("cnt_line_too_small:     %u\n", cnt_line_is_too_small);
  printf("line_usage:");
  uint64_t line_usage_total_nsamples = 0;
  for (uint32_t i = 0; i < FIFO_RX_LINE_MAX_SAMPLES; i++) {
    uint32_t line_usage = line_usage_histogram[i];
    line_usage_total_nsamples += (line_usage * i);
    printf(" %u", line_usage);
  }
  printf("\n");
  printf("line_usage_total_nsamples:  %llu\n", line_usage_total_nsamples);
  printf("------\n\n");
  pthread_mutex_unlock(&lock);

  if (!sensor_reader_break) {

    printf("wait for file_writer to terminate...\n");

    // wait for file writer up to duration seconds
    // checking every second if it is terminated
    for (uint32_t i = 0; i < duration; i++) {
      if (file_writer_terminated) break;
      else sleep(1);
    }

    // strange, why it is not terminated
    if (!file_writer_terminated) {

      printf("ERROR: file_writer thread did not terminate on its own after %u seconds.\n", duration);
      printf("       This should probably never happen unless file I/O is very slow.\n");
      printf("       There are probably missing samples at the end of the output file.\n");

    }

  }
    
  return NULL;

}

int main(int argc, char *argv[]) {

  // programmers check
  // there are multiple places this is assumed one byte
  assert(FIFO_RX_LINE_MAX_SAMPLES <= 0xFF);

  duration = 1;

  if (argc == 2) {
    duration = strtoul(argv[1], NULL, 10);
  }

  spi_configure();

  printf("SPI configured.\n");

  if (whoami() != 0x7B) {
    printf("not an IIS3DWB\n");
    quitf();
  }

  iis3dwb_swreset();
  iis3dwb_configure();
  iis3dwb_set_fullscale(FULL_SCALE);

  const double afactor = (2.0 * FULL_SCALE) / (1 << 16);
  const double tfactor = iis3dwb_timestamp_resolution_actual();

  printf("IIS3DWB configured.\n");

  dump_regs();

  const double odr = iis3dwb_odr_actual();
  // 2* because there is one timestamp and one accelerometer data
  nsamples = 2 * duration * odr;

  printf("num_fifo_rx_lines=%u, fifo_rx_line_max_samples=%u\n", 
      NUM_FIFO_RX_LINES, 
      FIFO_RX_LINE_MAX_SAMPLES);

  uint32_t fifo_rx_memory_usage = (sizeof(fifo_rx) / sizeof(fifo_rx[0])) / (1024*1024);
  printf("fifo_rx is %uMB\n", fifo_rx_memory_usage);

  printf("duration=%u seconds\n", duration);
  printf("nsamples=%llu\n\n", nsamples);

  signal(SIGINT, sig_handler);

  pthread_t tid_sensor_reader;
  pthread_create(&tid_sensor_reader, NULL, sensor_reader, NULL);

  pthread_t tid_file_writer;
  pthread_create(&tid_file_writer, NULL, file_writer, NULL);

  cpu_set_t core2;
  CPU_ZERO(&core2);
  CPU_SET(2, &core2);

  cpu_set_t core3;
  CPU_ZERO(&core3);
  CPU_SET(3, &core3);

  pthread_setaffinity_np(tid_sensor_reader, sizeof(cpu_set_t), &core2);
  pthread_setaffinity_np(tid_file_writer, sizeof(cpu_set_t), &core3);

  pthread_join(tid_sensor_reader, NULL);

  if (sensor_reader_break) {

    printf("breaking the file_writer to terminate...\n");
    file_writer_break = true;
    pthread_join(tid_file_writer, NULL);
    printf("ERROR. Do not use the output files.\n");
    quitf();

  } else {

    // append afactor and tfactor to the file
    FILE* fp = fopen(TEMP_FILE_META, "wb");
    fwrite(&duration, sizeof(uint32_t), 1, fp);
    fwrite(&nsamples, sizeof(uint64_t), 1, fp);
    const double fs = FULL_SCALE;
    fwrite(&fs, sizeof(double), 1, fp);
    fwrite(&odr, sizeof(double), 1, fp);
    fwrite(&afactor, sizeof(double), 1, fp);
    fwrite(&tfactor, sizeof(double), 1, fp);
    fclose(fp);

    printf("SUCCESS. Data collection finished. %s is OK.\n", TEMP_FILE);
    quits();

  }

}
