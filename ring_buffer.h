#ifndef __RING_BUF_H
#define __RING_BUF_H

#define SERIAL_BUFFER_SIZE 64

class ring_buffer {
 
public:
    unsigned char buffer[SERIAL_BUFFER_SIZE];
    unsigned char head;
    unsigned char tail;
    
    ring_buffer() {
        head = 0;
        tail = 0;
    }
 
    void write(unsigned char c) {
        int i = (unsigned char)(head + 1) % SERIAL_BUFFER_SIZE;

        // if we should be storing the received character into the location
        // just before the tail (meaning that the head would advance to the
        // current location of the tail), we're about to overflow the buffer
        // and so we don't write the character or advance the head.
        if (i != tail) {
            buffer[head] = c;
            head = i;
        }
    }
 
    int read(void) {
        // if the head isn't ahead of the tail, we don't have any characters
        if (head == tail) {
            return -1;
        } else {
            unsigned char c = buffer[tail];
            tail = (unsigned char)(tail + 1) % SERIAL_BUFFER_SIZE;
            return c;
        }
    }
    
    void reset(void) {
        head = tail;
    }
 
 
protected:

};

#endif
