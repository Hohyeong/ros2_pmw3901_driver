#ifndef PMW3901_HPP
#define PMW3901_HPP

#include <stdint.h>
#include <string>
#include <vector>

class PMW3901 {
  public:
    PMW3901(const std::string& spi_device, int cs_pin, int motion_pin);

    bool begin();
    void read_motion_count(int16_t* delta_x, int16_t* delta_y);
    void enable_frame_buffer();
    void read_frame_buffer(std::vector<uint8_t>& buffer);

  private:
    std::string spi_device;
    int cs_pin;
    int motion_pin;
    int spi_fd;

    struct gpiod_chip* chip;
    struct gpiod_line* cs_line;
    struct gpiod_line* motion_line;

    void cs_high();
    void cs_low();

    void register_write(uint8_t reg, uint8_t value);
    uint8_t register_read(uint8_t reg);
    void init_registers();

    uint8_t transfer_byte(uint8_t data);
};

constexpr uint8_t CHIP_ID = 0x49;
constexpr uint8_t CHIP_ID_INVERSE = 0xB6;

#endif // PMW3901_HPP
