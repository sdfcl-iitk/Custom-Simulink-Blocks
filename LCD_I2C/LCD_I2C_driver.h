#pragma once

#include <cinttypes>

#include <cstdint>

extern "C" {
	#include <unistd.h>
	#include <fcntl.h>
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
	#include <sys/ioctl.h>
}

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

/**
 * This is the driver for the Liquid Crystal LCD displays that use the I2C bus.
 *
 * After creating an instance of this class, first call begin() before anything else.
 * The backlight is on by default, since that is the most likely operating mode in
 * most cases.
 */
class LCD_I2C_driver {
private:
	int i2c_bus;
	uint8_t _addr;
	uint8_t _displayfunction;
	uint8_t _displaycontrol;
	uint8_t _displaymode;
	uint8_t _cols;
	uint8_t _rows;
	uint8_t _charsize;
	uint8_t _backlightval;

	/* ------------------------------------------
	    Liquid Crystal I2C commands
	   ------------------------------------------
	*/
	
	static constexpr uint8_t LCD_CLEARDISPLAY = 0x01;
	static constexpr uint8_t LCD_RETURNHOME = 0x02;
	static constexpr uint8_t LCD_ENTRYMODESET = 0x04;
	static constexpr uint8_t LCD_DISPLAYCONTROL = 0x08;
	static constexpr uint8_t LCD_CURSORSHIFT = 0x10;
	static constexpr uint8_t LCD_FUNCTIONSET = 0x20;
	static constexpr uint8_t LCD_SETCGRAMADDR = 0x40;
	static constexpr uint8_t LCD_SETDDRAMADDR = 0x80;

	// flags for display entry mode
	static constexpr uint8_t LCD_ENTRYRIGHT = 0x00;
	static constexpr uint8_t LCD_ENTRYLEFT = 0x02;
	static constexpr uint8_t LCD_ENTRYSHIFTINCREMENT = 0x01;
	static constexpr uint8_t LCD_ENTRYSHIFTDECREMENT = 0x00;

	// flags for display on/off control
	static constexpr uint8_t LCD_DISPLAYON = 0x04;
	static constexpr uint8_t LCD_DISPLAYOFF = 0x00;
	static constexpr uint8_t LCD_CURSORON = 0x02;
	static constexpr uint8_t LCD_CURSOROFF = 0x00;
	static constexpr uint8_t LCD_BLINKON = 0x01;
	static constexpr uint8_t LCD_BLINKOFF = 0x00;

	// flags for display/cursor shift
	static constexpr uint8_t LCD_DISPLAYMOVE = 0x08;
	static constexpr uint8_t LCD_CURSORMOVE = 0x00;
	static constexpr uint8_t LCD_MOVERIGHT = 0x04;
	static constexpr uint8_t LCD_MOVELEFT = 0x00;

	// flags for function set
	static constexpr uint8_t LCD_8BITMODE = 0x10;
	static constexpr uint8_t LCD_4BITMODE = 0x00;
	static constexpr uint8_t LCD_2LINE = 0x08;
	static constexpr uint8_t LCD_1LINE = 0x00;
	static constexpr uint8_t LCD_5x10DOTS = 0x04;
	static constexpr uint8_t LCD_5x8DOTS = 0x00;

	// flags for backlight control
	static constexpr uint8_t LCD_BACKLIGHT = 0x08;
	static constexpr uint8_t LCD_NOBACKLIGHT = 0x00;

	static constexpr uint8_t En = 0b00000100;  // Enable bit
	static constexpr uint8_t Rw = 0b00000010;  // Read/Write bit
	static constexpr uint8_t Rs = 0b00000001;  // Register select bit

	/* ------------------------------------------
	    Commands end
	   ------------------------------------------
	*/

public:
	/**
	 * Constructor
	 *
	 * @param lcd_addr	I2C slave address of the LCD display. Most likely printed on the
	 *					LCD circuit board, or look in the supplied LCD documentation.
	 * @param lcd_cols	Number of columns your LCD display has.
	 * @param lcd_rows	Number of rows your LCD display has.
	 * @param charsize	The size in dots that the display has, use LCD_5x10DOTS or LCD_5x8DOTS.
	 */
	LCD_I2C_driver(uint8_t lcd_addr=0x27, uint8_t lcd_cols=16, uint8_t lcd_rows=2, uint8_t charsize = LCD_5x8DOTS) {
		_addr = lcd_addr;
		_cols = lcd_cols;
		_rows = lcd_rows;
		_charsize = charsize;
		_backlightval = LCD_BACKLIGHT;

		i2c_bus = open("/dev/i2c-1", O_RDWR);
		ioctl(i2c_bus, I2C_SLAVE, _addr);
	}

	~LCD_I2C_driver() {
		// close the bus
		if (i2c_bus > 0) close(i2c_bus);
	}

	/**
	 * Set the LCD display in the correct begin state, must be called before anything else is done.
	 */
	void begin() {
		_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;

		if (_rows > 1) {
			_displayfunction |= LCD_2LINE;
		}

		// for some 1 line displays you can select a 10 pixel high font
		if ((_charsize != 0) && (_rows == 1)) {
			_displayfunction |= LCD_5x10DOTS;
		}

		// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
		// according to datasheet, we need at least 40ms after power rises above 2.7V
		// before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
		// ::sleep(50 / 1000);

		// Now we pull both RS and R/W low to begin commands
		expanderWrite(_backlightval);	// reset expanderand turn backlight off (Bit 8 =1)
		// ::sleep(1000 / 1000);

		//put the LCD into 4 bit mode
		// this is according to the hitachi HD44780 datasheet
		// figure 24, pg 46

		// we start in 8bit mode, try to set 4 bit mode
		write4bits(0x03 << 4);
		// ::sleep(4500 / 1e6); // wait min 4.1ms

		// second try
		write4bits(0x03 << 4);
		// ::sleep(4500 / 1e6); // wait min 4.1ms

		// third go!
		write4bits(0x03 << 4);
		// ::sleep(150 / 1e6);

		// finally, set to 4-bit interface
		write4bits(0x02 << 4);

		// set # lines, font size, etc.
		command(LCD_FUNCTIONSET | _displayfunction);

		// turn the display on with no cursor or blinking default
		_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
		display();

		// clear it off
		clear();

		// Initialize to default text direction (for roman languages)
		_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

		// set the entry mode
		command(LCD_ENTRYMODESET | _displaymode);

		home();
	}

	 /**
	  * Remove all the characters currently shown. Next print/write operation will start
	  * from the first position on LCD display.
	  */
	void clear() {
		command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
	}

	/**
	 * Next print/write operation will will start from the first position on the LCD display.
	 */
	void home() {
		command(LCD_RETURNHOME);  // set cursor position to zero
	}

	 /**
	  * Do not show any characters on the LCD display. Backlight state will remain unchanged.
	  * Also all characters written on the display will return, when the display in enabled again.
	  */
	void noDisplay() {
		_displaycontrol &= ~LCD_DISPLAYON;
		command(LCD_DISPLAYCONTROL | _displaycontrol);
	}

	/**
	 * Show the characters on the LCD display, this is the normal behaviour. This method should
	 * only be used after noDisplay() has been used.
	 */
	void display() {
		_displaycontrol |= LCD_DISPLAYON;
		command(LCD_DISPLAYCONTROL | _displaycontrol);
	}

	/**
	 * Do not blink the cursor indicator.
	 */
	void noBlink() {
		_displaycontrol &= ~LCD_BLINKON;
		command(LCD_DISPLAYCONTROL | _displaycontrol);
	}

	/**
	 * Start blinking the cursor indicator.
	 */
	void blink() {
		_displaycontrol |= LCD_BLINKON;
		command(LCD_DISPLAYCONTROL | _displaycontrol);
	}

	/**
	 * Do not show a cursor indicator.
	 */
	void noCursor() {
		_displaycontrol &= ~LCD_CURSORON;
		command(LCD_DISPLAYCONTROL | _displaycontrol);
	}

	/**
 	 * Show a cursor indicator, cursor can blink on not blink. Use the
	 * methods blink() and noBlink() for changing cursor blink.
	 */
	void cursor() {
		_displaycontrol |= LCD_CURSORON;
		command(LCD_DISPLAYCONTROL | _displaycontrol);
	}

	void scrollDisplayLeft() {
		command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
	}

	void scrollDisplayRight() {
		command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
	}

	// void printLeft();
	// void printRight();
	
	void leftToRight() {
		_displaymode |= LCD_ENTRYLEFT;
		command(LCD_ENTRYMODESET | _displaymode);
	}
	
	void rightToLeft() {
		_displaymode &= ~LCD_ENTRYLEFT;
		command(LCD_ENTRYMODESET | _displaymode);
	}
	
	// void shiftIncrement();
	// void shiftDecrement();
	
	void noBacklight() {
		_backlightval=LCD_NOBACKLIGHT;
		expanderWrite(0);
	}

	void backlight() {
		_backlightval=LCD_BACKLIGHT;
		expanderWrite(0);
	}
	
	bool getBacklight() {
		return _backlightval == LCD_BACKLIGHT;
	}
	
	void autoscroll() {
		_displaymode |= LCD_ENTRYSHIFTINCREMENT;
		command(LCD_ENTRYMODESET | _displaymode);
	}
	
	void noAutoscroll() {
		_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
		command(LCD_ENTRYMODESET | _displaymode);
	}
	
	void createChar(uint8_t location, uint8_t charmap[]) {
		location &= 0x7; // we only have 8 locations 0-7
		command(LCD_SETCGRAMADDR | (location << 3));
		for (int i=0; i<8; i++) {
			write(charmap[i]);
		}
	}
	
	void setCursor(uint8_t row, uint8_t col) {
		int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
		if (row > _rows) {
			row = _rows-1;    // we count rows starting w/0
		}
		command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
	}
	
	size_t write(uint8_t value) {
		send(value, Rs);
		return 1;
	}
	
	void command(uint8_t value) {
		send(value, 0);
	}

	inline void blink_on() { blink(); }
	inline void blink_off() { noBlink(); }
	inline void cursor_on() { cursor(); }
	inline void cursor_off() { noCursor(); }

// Compatibility API function aliases
	void setBacklight(uint8_t new_val) {				// alias for backlight() and nobacklight()
		if (new_val) {
			backlight();		// turn backlight on
		} else {
			noBacklight();		// turn backlight off
		}
	}
	
	void load_custom_character(uint8_t char_num, uint8_t *rows) {	// alias for createChar()
		createChar(char_num, rows);
	}
	
	void print(const char *c) {
		// print a null terminated string
		while (*c) write(*c++);
	}

private:
	void send(uint8_t value, uint8_t mode) {
		uint8_t highnib=value&0xf0;
		uint8_t lownib=(value<<4)&0xf0;
		write4bits((highnib)|mode);
		write4bits((lownib)|mode);
	}
	
	void write4bits(uint8_t value) {
		expanderWrite(value);
		pulseEnable(value);
	}
	
	void expanderWrite(uint8_t _data) {
		uint8_t b = _data;
		b = b | _backlightval;

		i2c_smbus_write_byte(i2c_bus, b);
	}
	
	void pulseEnable(uint8_t _data) {
		expanderWrite(_data | En);	// En high
		expanderWrite(_data & ~En);	// En low
	}
};
