/*
 * ram-tester.c
 * Тестер микросхем оперативной памяти
 *
 * Created: 04.10.2025 15:42:30
 * Author : Прижимин Николай
 *
 * Микроконтроллер ATMega32
 *
 * Прошивка программы + фьюзы
 *
 * Внешний кварц 16 МГц:
 * avrdude -c usbasp -p m32 -U flash:w:ram-tester.hex:i -U lfuse:w:0xFF:m -U hfuse:w:0xD9:m
 *
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <limits.h>

/* ==================== ОПИСАНИЕ КОНФИГУРАЦИИ ==================== */

// I2C-адрес дисплея
#define LCD_I2C_ADDR 0x27
#define LCD_BACKLIGHT 0x08

// Адрес ячейки памяти для сохранения выбранного типа микросхемы
#define EEPROM_MEM_TYPE_ADDR 0x00

// Определения пинов кнопок
#define BUTTON_SELECT_PIN   PC2
#define BUTTON_START_PIN    PC3

// I2C
#define I2C_SCL PC0
#define I2C_SDA PC1

// Пины для управления микросхемой памяти
#define OE_PIN	PC4
#define WE_PIN	PC5
#define VCC_PIN	PC6
#define CE_PIN	PC7

// Дополнительные пины для DRAM
/*
*
* Пины CAS_PIN и RAS_PIN пересекаются с OE_PIN и CAS_PIN без проблем,
* т.к. в один момент времени тестируется или статическая память, или динамическая.
*
*/
#define CAS_PIN PC4
#define RAS_PIN PC7
#define DRAM_DATA_PIN  PB0   // Выходы Data output (Dout) и Data input (Din) - соединены
#define DRAM_DATA  (1 << DRAM_DATA_PIN)

// Константы для временных параметров DRAM
#define DRAM_RAS_PRECHARGE_US   1
#define DRAM_CAS_DELAY_US       1
#define DRAM_REFRESH_INTERVAL	64

// Типы поддерживаемых микросхем памяти
typedef enum {
	MEM_6116 =  0,
	MEM_6164 =  1,
	MEM_6264 =  2,
	MEM_62256 = 3,
	MEM_2118 =  4,
	MEM_4164 =  5,
	MEM_41256 = 6,
	MEM_COUNT = 7
} memory_type_t;

// Структура конфигурации памяти
typedef struct {
	uint16_t size_kb;
	uint8_t addr_lines;
	uint8_t is_dip28;
	uint8_t ce_config;
	uint8_t use_a13_as_cs;
	uint8_t is_dram;
	uint8_t addr_multiplex;
} memory_config_t;

// Таблица конфигураций памяти
const memory_config_t mem_config[] = {
	// size,	addr,	dip28,	ce,	a13_as_cs,	dram,	mux
	{ 2,		11,		0,		1,	0,			0,		0 }, // 6116
	{ 8,		13,		1,		1,	1,			0,		0 }, // 6164
	{ 8,		13,		1,		1,	1,			0,		0 }, // 6264
	{ 32,		15,		1,		0,	0,			0,		0 }, // 62256
	{ 2,		14,		0,		0,	0,			1,		7 }, // 2118
	{ 8,		16,		0,		0,	0,			1,		8 }, // 4164
	{ 32,		18,		0,		0,	0,			1,		9 }  // 41256
};

volatile memory_type_t current_mem_type = MEM_4164;
volatile uint8_t test_running = 0;

// Прототипы функций

// Управление кнопками
uint8_t is_button_pressed(uint8_t pin);

// Работа с EEPROM
memory_type_t read_memory_type_from_eeprom(void);
void save_memory_type_to_eeprom(memory_type_t type);
void cycle_memory_type(void);

// I2C и LCD
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t data);
void lcd_send_byte(uint8_t data, uint8_t mode);
void lcd_init(void);
void lcd_print(const char *str);
void lcd_print_at(uint8_t col, uint8_t row, const char *str);
void lcd_clear(void);
void display_current_memory_info(void);

// Инициализация системы
void system_init(void);

// Управление памятью
void configure_memory_controller(memory_type_t type);
void power_on_memory(void);
void power_off_memory(void);

// Информация о памяти
uint16_t get_memory_size(memory_type_t type);
uint8_t get_address_lines(memory_type_t type);
uint8_t is_valid_address(memory_type_t type, uint16_t address);
void get_dram_address_bits(memory_type_t type, uint8_t* row_bits, uint8_t* col_bits);

// Функции для SRAM
void sram_memory_write(uint16_t address, uint8_t data);
uint8_t sram_memory_read(uint16_t address);

// Функции для DRAM
void configure_dram_ports(void);
void dram_write_bit(uint8_t row, uint8_t col, uint8_t data_bit);
uint8_t dram_read_bit(uint8_t row, uint8_t col);

// Тестирование
void run_memory_tests(memory_type_t type);

// Тестирование SRAM
void run_sram_tests(memory_type_t type);
uint16_t sram_test1(memory_type_t type);
uint16_t sram_test2(memory_type_t type);
uint16_t sram_test3(memory_type_t type);
uint16_t sram_test4(memory_type_t type);

// Тестирование DRAM
void run_dram_tests(memory_type_t type);
uint16_t dram_test_single_bit(memory_type_t type);
uint16_t dram_test_alternating(memory_type_t type);

// Отображение результатов
void test_failed_message(uint8_t test, uint16_t error_address, uint8_t error_written, uint8_t error_read);
void test_complete_message(uint8_t test_number, uint16_t address);
void display_progress(uint16_t address);


/* ==================== РЕАЛИЗАЦИЯ ФУНКЦИЙ ==================== */

// Проверка нажатия кнопки с антидребезгом
uint8_t is_button_pressed(uint8_t pin) {
	if (!(PINC & (1 << pin))) {
		_delay_ms(30);
		if (!(PINC & (1 << pin))) {
			while (!(PINC & (1 << pin)));
			_delay_ms(20);
			return 1;
		}
	}
	return 0;
}

// Чтение типа памяти из EEPROM
memory_type_t read_memory_type_from_eeprom(void) {
	uint8_t saved_type = eeprom_read_byte((uint8_t*)EEPROM_MEM_TYPE_ADDR);
	return (saved_type < MEM_COUNT) ? (memory_type_t)saved_type : MEM_6116;
}

// Сохранение типа памяти в EEPROM
void save_memory_type_to_eeprom(memory_type_t type) {
	if (type < MEM_COUNT) {
		eeprom_write_byte((uint8_t*)EEPROM_MEM_TYPE_ADDR, (uint8_t)type);
	}
}

// Циклическое переключение типа памяти
void cycle_memory_type(void) {
	current_mem_type = (memory_type_t)((current_mem_type + 1) % MEM_COUNT);
	save_memory_type_to_eeprom(current_mem_type);
}

// I2C: начало передачи
void i2c_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

// I2C: окончание передачи
void i2c_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

// I2C: передача байта
void i2c_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

// Отправка байта на LCD
void lcd_send_byte(uint8_t data, uint8_t mode) {
	uint8_t high = data & 0xF0;
	uint8_t low = (data << 4) & 0xF0;
	uint8_t data_arr[4];

	data_arr[0] = high | mode | LCD_BACKLIGHT | 0x04;
	data_arr[1] = high | mode | LCD_BACKLIGHT;
	data_arr[2] = low | mode | LCD_BACKLIGHT | 0x04;
	data_arr[3] = low | mode | LCD_BACKLIGHT;

	i2c_start();
	i2c_write(LCD_I2C_ADDR << 1);

	for(uint8_t i = 0; i < 4; i++) {
		i2c_write(data_arr[i]);
		_delay_us(100);
	}

	i2c_stop();
	_delay_us(100);
}

// Инициализация LCD
void lcd_init(void) {
	_delay_ms(50);
	lcd_send_byte(0x33, 0);
	_delay_ms(5);
	lcd_send_byte(0x32, 0);
	_delay_ms(5);
	lcd_send_byte(0x28, 0);
	_delay_ms(1);
	lcd_send_byte(0x0C, 0);
	_delay_ms(1);
	lcd_send_byte(0x01, 0);
	_delay_ms(2);
	lcd_send_byte(0x06, 0);
	_ddelay_ms(1);
}

// Вывод строки на LCD
void lcd_print(const char *str) {
	while (*str) {
		lcd_send_byte(*str++, 1);
	}
}

// Вывод строки на LCD в указанную позицию
void lcd_print_at(uint8_t col, uint8_t row, const char *str) {
	uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	if (row >= 4) row = 3;
	lcd_send_byte(0x80 | (col + row_offsets[row]), 0);
	lcd_print(str);
}

// Очистка LCD
void lcd_clear(void) {
	lcd_send_byte(0x01, 0);
	_delay_ms(2);
}

// Отображение информации о текущем типе памяти
void display_current_memory_info(void) {
	const char* mem_names[] = {
		"6116 (2Kx8)  ", "6164 (8Kx8)  ", "6264 (8Kx8)  ", "62256 (32Kx8)",
		"2118 (16Kx1)", "4164 (64Kx1)  ", "41256 (256Kx1)"
	};

	lcd_clear();
	lcd_print_at(0, 0, "RAM Tester v1.1     ");
	lcd_print_at(0, 1, "Type:               ");
	lcd_print_at(5, 1, mem_names[current_mem_type]);

	if (test_running) {
		lcd_print_at(0, 2, "Status: TESTING  ");
		lcd_print_at(0, 3, "Press STOP to end");
	} else {
		lcd_print_at(0, 2, "Status: READY       ");
		lcd_print_at(0, 3, "SEL:Type START:Test ");
	}
}

// Инициализация системы
void system_init(void) {
	DDRA = 0xFF;
	DDRB = 0xFF;
	DDRD = 0xFF;
	DDRC = 0xF3;

	PORTA = 0x00;
	PORTB = 0x00;
	PORTD = 0x00;
	PORTC = (1 << BUTTON_SELECT_PIN) | (1 << BUTTON_START_PIN);

	PORTC |= (1 << CE_PIN) | (1 << WE_PIN) | (1 << OE_PIN);
	power_off_memory();

	// I2C 100 КГц
	TWSR = 0x00;
	TWBR = 72;
	TWCR = (1 << TWEN);
}

// Конфигурация контроллера памяти для выбранного типа
void configure_memory_controller(memory_type_t type) {
	memory_config_t cfg = mem_config[type];

	if (cfg.is_dram) {
		PORTC |= (1 << RAS_PIN) | (1 << CAS_PIN) | (1 << WE_PIN);
	} else {
		PORTC |= (1 << WE_PIN) | (1 << OE_PIN);

		if(cfg.ce_config == 0x01) {
			PORTC |= (1 << CE_PIN);
		} else {
			PORTC &= ~(1 << CE_PIN);
		}

		if(!cfg.is_dip28) {
			PORTD &= 0xF0;
		}
	}
}

// Включение питания микросхемы памяти
void power_on_memory(void) {
	PORTC |= (1 << VCC_PIN);
	_delay_ms(200);
}

// Выключение питания микросхемы памяти
void power_off_memory(void) {
	PORTA = 0;
	PORTB = 0;
	PORTD = 0;
	PORTC &= ~((1 << VCC_PIN) | (1 << OE_PIN) | (1 << CE_PIN) | (1 << WE_PIN));
}

// Получение размера памяти в байтах
uint16_t get_memory_size(memory_type_t type) {
	return mem_config[type].size_kb * 1024;
}

// Получение количества адресных линий
uint8_t get_address_lines(memory_type_t type) {
	return mem_config[type].addr_lines;
}

// Проверка валидности адреса для данного типа памяти
uint8_t is_valid_address(memory_type_t type, uint16_t address) {
	uint16_t max_address = (1 << get_address_lines(type)) - 1;
	return address <= max_address;
}

// Получение количества бит для строк и столбцов DRAM
void get_dram_address_bits(memory_type_t type, uint8_t* row_bits, uint8_t* col_bits) {
    uint8_t mux_value = mem_config[type].addr_multiplex;
    *row_bits = mux_value;
    *col_bits = mux_value;
}

// Запись данных в память
void sram_memory_write(uint16_t address, uint8_t data) {
	if (mem_config[current_mem_type].use_a13_as_cs) {
		address |= (1 << 13);
	}

	PORTA = address & 0xFF;

	if (mem_config[current_mem_type].is_dip28) {
		PORTD = (address >> 8) & 0x7F;
	} else {
		PORTD = (PORTD & 0xF0) | ((address >> 8) & 0x0F);
	}

	if (mem_config[current_mem_type].ce_config == 0x01) {
		PORTC &= ~(1 << CE_PIN);
	}

	PORTB = data;
	_delay_us(10);

	PORTC &= ~(1 << WE_PIN);
	_delay_us(10);
	PORTC |= (1 << WE_PIN);
	_delay_us(10);

	if (mem_config[current_mem_type].ce_config == 0x01) {
		PORTC |= (1 << CE_PIN);
	}
}

// Чтение данных из памяти
uint8_t sram_memory_read(uint16_t address) {
	uint8_t data;

	if (mem_config[current_mem_type].use_a13_as_cs) {
		address |= (1 << 13);
	}

	PORTA = address & 0xFF;

	if (mem_config[current_mem_type].is_dip28) {
		PORTD = (address >> 8) & 0x7F;
	} else {
		PORTD = (PORTD & 0xF0) | ((address >> 8) & 0x0F);
	}

	if (mem_config[current_mem_type].ce_config == 0x01) {
		PORTC &= ~(1 << CE_PIN);
	}

	DDRB = 0x00;
	PORTB = 0xFF;
	_delay_us(5);

	PORTC &= ~(1 << OE_PIN);
	_delay_us(10);
	data = PINB;
	PORTC |= (1 << OE_PIN);

	if (mem_config[current_mem_type].ce_config == 0x01) {
		PORTC |= (1 << CE_PIN);
	}

	DDRB = 0xFF;
	PORTB = 0x00;

	return data;
}

// Конфигурация портов для работы с DRAM
void configure_dram_ports(void) {
    // Настройка портов для адресов строк и столбцов
    DDRA = 0xFF;  // PORTA - младшие 8 бит адреса (мультиплексированные строки/столбцы)
	PORTA = 0x00;
    DDRD = 0xFF;  // PORTD - старшие биты адреса (если нужны для больших DRAM)
	PORTD = 0X00;

    // Настройка порта данных
    DDRB |= DRAM_DATA;  // PB0 как выход для записи
	PORTB &= ~DRAM_DATA;  // По умолчанию 0

    // Настройка управляющих сигналов
    DDRC |= (1 << RAS_PIN) | (1 << CAS_PIN) | (1 << WE_PIN);

    // Установка начального состояния сигналов
    PORTC |= (1 << RAS_PIN) | (1 << CAS_PIN) | (1 << WE_PIN);
}

// Запись бита в DRAM
void dram_write_bit(uint8_t row, uint8_t col, uint8_t data_bit) {
    // Настройка порта данных на выход
    DDRB |= DRAM_DATA;

    // Подача адреса строки
    PORTA = row;

    // Активация RAS (защелкиваем адрес строки)
    PORTC &= ~(1 << RAS_PIN);
    asm volatile ("nop");  // t_RCD = 20-30 нс (RAS to CAS delay)

    // Подача адреса столбца
    PORTA = col;

    // Установка данных
    if (data_bit) {
        PORTB |= DRAM_DATA;
    } else {
        PORTB &= ~DRAM_DATA;
    }

    // Активация CAS и WE (цикл записи)
    PORTC &= ~((1 << CAS_PIN) | (1 << WE_PIN));
    asm volatile ("nop");  // t_WP = 20-30 нс (Write pulse width)

    // Деактивация CAS и WE
    PORTC |= (1 << CAS_PIN) | (1 << WE_PIN);
    asm volatile ("nop");  // t_DH = 10 нс (Data hold time)

    // Деактивация RAS
    PORTC |= (1 << RAS_PIN);
    asm volatile ("nop");  // t_RAS = 60-100 нс (RAS pulse width)
}

// Чтение бита из DRAM
uint8_t dram_read_bit(uint8_t row, uint8_t col) {
    uint8_t data_bit;

    // Настройка порта данных на вход
    DDRB &= ~DRAM_DATA;
    PORTB &= ~DRAM_DATA; // отключить подтяжку

    // Подача адреса строки
    PORTA = row;

    // Активация RAS (защелкиваем адрес строки)
    PORTC &= ~(1 << RAS_PIN);
    asm volatile ("nop");  // t_RCD = 20-30 нс (RAS to CAS delay)

    // Подача адреса столбца
    PORTA = col;

    // Активация CAS (цикл чтения)
    PORTC &= ~(1 << CAS_PIN);
    asm volatile ("nop");  // t_CAC = 20-30 нс (CAS access time)

    // Чтение данных
    data_bit = (PINB & DRAM_DATA) ? 1 : 0;

    // Деактивация CAS
    PORTC |= (1 << CAS_PIN);
    asm volatile ("nop");  // t_OFF = 10 нс (Output disable time)

    // Деактивация RAS
    PORTC |= (1 << RAS_PIN);
    asm volatile ("nop");  // t_RP = 20-30 нс (RAS precharge time)

    return data_bit;
}

// Запуск всех тестов памяти
void run_memory_tests(memory_type_t type) {
    configure_memory_controller(type);
    power_on_memory();

    if (mem_config[type].is_dram) {
        run_dram_tests(type);
    } else {
		run_sram_tests(type);
    }
}

// Тест 1: запись и чтение шаблона 0x55
uint16_t sram_test1(memory_type_t type) {
	uint16_t memory_size = get_memory_size(type);

	lcd_print_at(0, 2, "Test 1: 0x55        ");
	for (uint16_t address = 0; address < memory_size; address++) {
		if (!is_valid_address(type, address)) continue;
		sram_memory_write(address, 0x55);
		uint8_t read_data = sram_memory_read(address);
		if (read_data != 0x55) {
			test_failed_message(1, address, 0x55, read_data);
			return address;
		}
		if ((address & 0xFF) == 0) {
            display_progress(address);
		}
	}
	test_complete_message(1, memory_size - 1);
	return UINT_MAX;
}

// Тест 2: запись и чтение шаблона 0xAA
uint16_t sram_test2(memory_type_t type) {
    uint16_t memory_size = get_memory_size(type);

    lcd_print_at(0, 2, "Test 2: 0xAA        ");
    for (uint16_t address = 0; address < memory_size; address++) {
        if (!is_valid_address(type, address)) continue;
        sram_memory_write(address, 0xAA);
        uint8_t read_data = sram_memory_read(address);
        if (read_data != 0xAA) {
            test_failed_message(2, address, 0xAA, read_data);
            return address;
        }
        if ((address & 0xFF) == 0) {
            display_progress(address);
        }
    }
    test_complete_message(2, memory_size - 1);
    return UINT_MAX;
}

// Тест 3: проверка адресозависимости
uint16_t sram_test3(memory_type_t type) {
    uint16_t memory_size = get_memory_size(type);

    lcd_print_at(0, 2, "Test 3: AddrPat     ");
    for (uint16_t address = 0; address < memory_size; address++) {
        if (!is_valid_address(type, address)) continue;
        uint8_t pattern = address & 0xFF;
        sram_memory_write(address, pattern);
        uint8_t read_data = sram_memory_read(address);
        if (read_data != pattern) {
            test_failed_message(3, address, pattern, read_data);
            return address;
        }
        if ((address & 0xFF) == 0) {
            display_progress(address);
        }
    }
    test_complete_message(3, memory_size - 1);
    return UINT_MAX;
}

// Тест 4: проверка с инверсным шаблоном
uint16_t sram_test4(memory_type_t type) {
    uint16_t memory_size = get_memory_size(type);

    lcd_print_at(0, 2, "Test 4: InvPat      ");
    for (uint16_t address = 0; address < memory_size; address++) {
        if (!is_valid_address(type, address)) continue;
        uint8_t pattern = ~(address & 0xFF);
        sram_memory_write(address, pattern);
        uint8_t read_data = sram_memory_read(address);
        if (read_data != pattern) {
            test_failed_message(4, address, pattern, read_data);
            return address;
        }
        if ((address & 0xFF) == 0) {
            display_progress(address);
        }
    }
    test_complete_message(4, memory_size - 1);
    return UINT_MAX;
}

// Запуск тестов для SRAM памяти
void run_sram_tests(memory_type_t type) {
    lcd_print_at(0, 2, "Testing memory...   ");
    lcd_print_at(0, 3, "Please wait...      ");
    _delay_ms(500);

    uint16_t (*test_functions[])(memory_type_t) = {
        sram_test1, sram_test2, sram_test3, sram_test4
    };

    for (uint8_t i = 0; i < 4; i++) {
        if (test_functions[i](type) != UINT_MAX) {
            return;
        }
    }
    power_off_memory();
    lcd_print_at(0, 2, "SRAM TESTS PASSED!  ");
    lcd_print_at(0, 3, "Memory OK!          ");
    _delay_ms(3000);
}

// Тест DRAM: запись и чтение единиц
uint16_t dram_test_single_bit(memory_type_t type) {
    uint8_t row_bits, col_bits;
    get_dram_address_bits(type, &row_bits, &col_bits);

    uint8_t max_row = (1 << row_bits);
    uint8_t max_col = (1 << col_bits);

    lcd_print_at(0, 2, "DRAM Test: Bit 1    ");

    for (uint8_t row = 0; row < max_row; row++) {
        for (uint8_t col = 0; col < max_col; col++) {
            // Запись 1
            dram_write_bit(row, col, 1);
            uint8_t read_data = dram_read_bit(row, col);

            if (read_data != 1) {
                uint16_t error_addr = (row << 8) | col;
                test_failed_message(1, error_addr, 1, read_data);
                return error_addr;
            }
        }
        // Прогресс
        if ((row & 0x0F) == 0) {
            display_progress(row << 8);
        }
    }
    test_complete_message(1, (max_row << 8) | max_col);
    return UINT_MAX;
}

// Тест DRAM: чередующиеся 0 и 1
uint16_t dram_test_alternating(memory_type_t type) {
    uint8_t row_bits, col_bits;
    get_dram_address_bits(type, &row_bits, &col_bits);

    uint8_t max_row = (1 << row_bits);
    uint8_t max_col = (1 << col_bits);

    lcd_print_at(0, 2, "DRAM Test: 0/1 Alt ");

    for (uint8_t row = 0; row < max_row; row++) {
        for (uint8_t col = 0; col < max_col; col++) {
            uint8_t expected = (row + col) & 0x01;  // Чередование 0/1
            dram_write_bit(row, col, expected);
            uint8_t read_data = dram_read_bit(row, col);

            if (read_data != expected) {
                uint16_t error_addr = (row << 8) | col;
                test_failed_message(2, error_addr, expected, read_data);
                return error_addr;
            }
        }
        if ((row & 0x0F) == 0) {
            display_progress(row << 8);
        }
    }
    test_complete_message(2, (max_row << 8) | max_col);
    return UINT_MAX;
}

// Запуск тестов для DRAM памяти
void run_dram_tests(memory_type_t type) {
    configure_dram_ports();
    power_on_memory();

    lcd_print_at(0, 2, "Testing DRAM...     ");
    lcd_print_at(0, 3, "Please wait...      ");
    _delay_ms(50);

    // Просто запускаем тесты последовательно
    if (dram_test_single_bit(type) != UINT_MAX) {
		power_off_memory();
        return;
    }

    if (dram_test_alternating(type) != UINT_MAX) {
		power_off_memory();
        return;
    }

	power_off_memory();
	lcd_print_at(0, 2, "DRAM TESTS PASSED!  ");
    lcd_print_at(0, 3, "Memory OK!          ");
    _delay_ms(3000);
}

// Отображение прогресса тестирования
void display_progress(uint16_t address) {
    char progress_buf[6];
    sprintf(progress_buf, "%04X", address);
    lcd_print_at(16, 2, progress_buf);
}

// Вывод сообщения об успешном завершении теста
void test_complete_message(uint8_t test_number, uint16_t address) {
    char pass_buf[20];
    sprintf(pass_buf, "Test %d: PASS    %04X", test_number, address);
    lcd_print_at(0, 2, pass_buf);
	_delay_ms(500);
}

// Вывод сообщения об ошибке теста
void test_failed_message(uint8_t test, uint16_t error_address, uint8_t error_written, uint8_t error_read) {
	power_off_memory();
	char test_line[20];
	sprintf(test_line, "Test %d FAILED       ", test);
	lcd_print_at(0, 2, test_line);

	char buf[17];
	sprintf(buf, "A:%04X W:%02X R:%02X", error_address, error_written, error_read);
	lcd_print_at(0, 3, buf);
	_delay_ms(3000);
}

// Главная функция программы
int main(void) {
	system_init();
	lcd_init();

	current_mem_type = read_memory_type_from_eeprom();
	memory_type_t previous_mem_type = current_mem_type;

	display_current_memory_info();

	while (1) {
		if (current_mem_type != previous_mem_type) {
			display_current_memory_info();
			previous_mem_type = current_mem_type;
		}

		if (is_button_pressed(BUTTON_SELECT_PIN)) {
			cycle_memory_type();
			lcd_print_at(0, 3, "Type changed!       ");
			_delay_ms(500);
		}

		if (is_button_pressed(BUTTON_START_PIN)) {
			if (!test_running) {
				test_running = 1;
				save_memory_type_to_eeprom(current_mem_type);
				run_memory_tests(current_mem_type);
				test_running = 0;
				previous_mem_type = current_mem_type;
				display_current_memory_info();
			}
		}
		_delay_ms(10);
	}
}