//
// Created by timmy on 1/10/21.
//
#include <Leg.h>

void leg::Step_fw(int val) {
    this->s1.write(val + this->f_counter);
    this->f_counter = (this->f_counter +1) % 46;
}

void leg::Step_bw(int val) {{
        this->s1.write(val - this->b_counter);
        this->b_counter = (this->b_counter+1) % 46;
}}
