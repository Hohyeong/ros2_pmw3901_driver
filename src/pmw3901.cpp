#include "pmw3901.hpp"
#include <gpiod.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono_literals;


std::string spi_device;
int cs_pin, motion_pin;
int spi_fd;

struct gpiod_chip* chip;
struct gpiod_line* cs_line;
struct gpiod_line* motion_line;

PMW3901::PMW3901(const std::string& spi_device, int cs_pin, int motion_pin)
  : spi_device(spi_device), cs_pin(cs_pin), motion_pin(motion_pin) {}

bool PMW3901::begin() {
  spi_fd = open(spi_device.c_str(), O_RDWR);
  if (spi_fd < 0) {
    perror("SPI open failed");
    return false;
  }
  uint8_t mode = SPI_MODE_3;
  ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
  uint32_t speed = 2000000; // 2MHz
  ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

  chip = gpiod_chip_open_by_name("gpiochip0");
  if (!chip) { perror("gpiod_chip_open"); return false; }

  cs_line = gpiod_chip_get_line(chip, cs_pin);
  motion_line = gpiod_chip_get_line(chip, motion_pin);
  if (!cs_line || !motion_line) { perror("gpiod_line_get"); return false; }

  // CS: initial HIGH
  if (gpiod_line_request_output(cs_line, "pmw3901", 1) < 0) {
    perror("gpiod_line_request_output cs"); return false;
  }
  // MOTION:
  if (gpiod_line_request_input(motion_line, "pmw3901") < 0) {
    perror("gpiod_line_request_input motion"); return false;
  }

  // --- Reset ---
  cs_high(); std::this_thread::sleep_for(1ms);
  cs_low();  std::this_thread::sleep_for(1ms);
  cs_high(); std::this_thread::sleep_for(1ms);

  register_write(0x3A, 0x5A); // Power-up reset
  std::this_thread::sleep_for(5ms);

  uint8_t chip_id = register_read(0x00);
  uint8_t chip_id_inv = register_read(0x5F);

  std::cout << "! PMW3901 connection established(PMW3901 ChipID: 0x" << std::hex << int(chip_id)
            << ")" << std::endl;

  if (chip_id != CHIP_ID || chip_id_inv != CHIP_ID_INVERSE) return false;

  register_read(0x02); register_read(0x03);
  register_read(0x04); register_read(0x05); register_read(0x06);
  std::this_thread::sleep_for(1ms);

  init_registers();

  return true;
}

void PMW3901::cs_high() { gpiod_line_set_value(cs_line, 1); }
void PMW3901::cs_low()  { gpiod_line_set_value(cs_line, 0); }

uint8_t PMW3901::register_read(uint8_t reg) {
  reg &= ~0x80u;
  cs_low();
  uint8_t tx[2] = { reg, 0 };
  uint8_t rx[2] = {0};
  struct spi_ioc_transfer tr{};
  tr.tx_buf = (unsigned long)tx;
  tr.rx_buf = (unsigned long)rx;
  tr.len = 2;
  tr.speed_hz = 2000000;
  tr.bits_per_word = 8;
  tr.delay_usecs = 0;
  ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
  cs_high();
  return rx[1];
}

void PMW3901::register_write(uint8_t reg, uint8_t val) {
  reg |= 0x80u;
  cs_low();
  uint8_t buf[2] = { reg, val };
  struct spi_ioc_transfer tr{};
  tr.tx_buf = (unsigned long)buf;
  tr.rx_buf = 0;
  tr.len = 2;
  tr.speed_hz = 2000000;
  tr.bits_per_word = 8;
  tr.delay_usecs = 0;
  ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
  cs_high();
  std::this_thread::sleep_for(20us);
}

void PMW3901::read_motion_count(int16_t* delta_x, int16_t* delta_y) {
  register_read(0x02);
  *delta_x = ((int16_t)register_read(0x04) << 8) | register_read(0x03);
  *delta_y = ((int16_t)register_read(0x06) << 8) | register_read(0x05);
}

void PMW3901::enable_frame_buffer() {
  register_write(0x7F, 0x07);
  register_write(0x4C, 0x00);
  register_write(0x7F, 0x08);
  register_write(0x6A, 0x38);
  register_write(0x7F, 0x00);
  register_write(0x55, 0x04);
  register_write(0x40, 0x80);
  register_write(0x4D, 0x11);

  std::this_thread::sleep_for(10ms);

  register_write(0x7F, 0x00);
  register_write(0x58, 0xFF);

  uint8_t temp;
  int timeout = 100000;
  do {
    temp = register_read(0x59);
  } while(--timeout > 0 && temp >> 6 == 0x00);

  if(timeout == 0) std::cout << "[WARN] FrameInit timeout." << std::endl;
  register_write(0x58, 0x00);
}

void PMW3901::read_frame_buffer(std::vector<uint8_t>& buffer) {
  buffer.resize(35*35);
  for(int i=0;i<35*35;i++) {
    uint8_t data;
    int timeout = 100000;

    do {
      data = register_read(0x58);

      if(data >> 6 == 0x01) {
        buffer[i] &= ~0b11111100;
        buffer[i] |= (data & 0b00111111) << 2;
      }
      if(data >> 6 == 0x02) {
        buffer[i] &= ~0b00000011;
        buffer[i] |= (data & 0b00001100) >> 2;
        break;
      }
    } while(--timeout > 0);

    if(timeout == 0) {
      std::cout << "[WARN] FrameRead timeout." << std::endl;
      break;
    }
  }
}

void PMW3901::init_registers() {
  register_write(0x7F, 0x00);
  register_write(0x61, 0xAD);
  register_write(0x7F, 0x03);
  register_write(0x40, 0x00);
  register_write(0x7F, 0x05);
  register_write(0x41, 0xB3);
  register_write(0x43, 0xF1);
  register_write(0x45, 0x14);
  register_write(0x5B, 0x32);
  register_write(0x5F, 0x34);
  register_write(0x7B, 0x08);
  register_write(0x7F, 0x06);
  register_write(0x44, 0x1B);
  register_write(0x40, 0xBF);
  register_write(0x4E, 0x3F);
}

