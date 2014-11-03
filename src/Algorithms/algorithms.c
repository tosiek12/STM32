#include "algorithms.h"

//funkcja dla ktorej obliczamy calke
float func(float x) {
	return x*x+3;
}

/*
 * xp - Poczatek przedzialu calkowania
 * xk - Koniec przedzialu calkowania
 * numberOfIntervals - ilosc podprzedzialow do sumy
 */
float integralTrapezoid(float xp, float xk, uint8_t numberOfIntervals) {
	float integral = 0, dx = 0;
	uint8_t i = 1;

	dx = (xk - xp) / (float) numberOfIntervals;
	for (i = 1; i < numberOfIntervals; i++) {
		integral += func(xp + i * dx);
	}
	integral += (func(xp) + func(xk)) / 2;	//chyba by uniknac bledow po dzieleniu
	integral *= dx;

	return integral;

}

float integralSimpson(float xp, float xk, uint8_t numberOfIntervals) {
	float integral = 0, dx = 0,s = 0, x = 0;
	uint8_t i = 1;

	dx = (xk - xp) / (float) numberOfIntervals;
	for (i = 1; i < numberOfIntervals; i++) {
		x = xp + i*dx;
		s += func(x - dx / 2);
		integral += func(x);
	}
	s += func(xk - dx / 2);
	integral = (dx/6) * (func(xp) + func(xk) + 2*integral + 4*s);
	return integral;

}
