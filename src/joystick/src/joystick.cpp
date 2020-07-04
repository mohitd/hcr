/*
 * Copyright (c) 2020. Mohit Deshpande.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "joystick/joystick.h"

#include <fcntl.h>
#include <unistd.h>

#define JS_EVENT_BUTTON 0x01    /* button pressed/released */
#define JS_EVENT_AXIS   0x02    /* joystick moved */
#define JS_EVENT_INIT   0x80    /* initial state of device */

namespace joystick {

// see https://www.kernel.org/doc/Documentation/input/joystick-api.txt
struct JoystickEvent {
    unsigned int time;
    short value;
    unsigned char type;
    unsigned char number;
};

JoystickInterface::JoystickInterface() {
    if ((joystick_fd_ = open("/dev/input/js0", O_RDONLY | O_NONBLOCK)) < 0) {
        throw std::invalid_argument("Unable to open joystick!");
    }
}

JoystickInterface::~JoystickInterface() {
    close(joystick_fd_);
}

bool JoystickInterface::Sample(JoystickValues& values) {
    JoystickEvent js_event{};
    int bytes_read = read(joystick_fd_, &js_event, sizeof(js_event));
    if (bytes_read != sizeof(js_event)) {
        return false;
    }

    if ((js_event.type & JS_EVENT_AXIS) != 0) {
        if (js_event.number == 0) {
            values.axis_1.x = -js_event.value;
        } else if (js_event.number == 1) {
            values.axis_1.y = -js_event.value;
        }
    }

    return true;
}

}
