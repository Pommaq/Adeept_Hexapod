//
// Created by timmy on 1/10/21.
//
#include <Leg.h>

void leg::Step_fw(double val, int modulo) {
    this->s1.write(val + this->f_counter);
    this->f_counter = (this->f_counter +1) % modulo;
}

void leg::Step_bw(double val, int modulo) {{
        this->s1.write(val - this->b_counter);
        this->b_counter = (this->b_counter+1) % modulo;
}}
