#include "main.h"


//Empty constructor
Simple_Moving_Average::Simple_Moving_Average() {}

void Simple_Moving_Average::set_period(int period) { 
  this->period = period;
  sum = 0;
}

void Simple_Moving_Average::add_data(double num) {
  sum += num;
  dataset.push(num);

  //Update size so that length of data set should be equal to period as a normal mean has
  if (dataset.size() > period) {
    sum -= dataset.front();
    dataset.pop();
  }
}

double Simple_Moving_Average::get_mean() { return sum / period; }