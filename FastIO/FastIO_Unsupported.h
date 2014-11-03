#include "mbed.h"

typedef struct {
    DigitalInOut *_pin;
} fastio_vars;

#define INIT_PIN        container._pin = new DigitalInOut(pin)
#define DESTROY_PIN     delete(container._pin)

#define SET_DIR_INPUT   container._pin->input()
#define SET_DIR_OUTPUT  container._pin->output()
#define SET_MODE(pull)  container._pin->mode(pull)

#define WRITE_PIN_SET   container._pin->write(1)
#define WRITE_PIN_CLR   container._pin->write(0)

#define READ_PIN        container._pin->read()

