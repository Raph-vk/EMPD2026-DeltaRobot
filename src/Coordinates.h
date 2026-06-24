/*
 * Coordinates.h
 *
 * Vaste pick-and-place-coordinaten in millimeters.
 */

#ifndef COORDINATES_H_
#define COORDINATES_H_

typedef struct
{
	float x;
	float y;
} Coordinate_t;

/* Middenplaat (bouten). */
extern const Coordinate_t M00;
extern const Coordinate_t M01;
extern const Coordinate_t M02;
extern const Coordinate_t M03;
extern const Coordinate_t M04;
extern const Coordinate_t M05;
extern const Coordinate_t M06;
extern const Coordinate_t M07;
extern const Coordinate_t M08;
extern const Coordinate_t M09;
extern const Coordinate_t M10;
extern const Coordinate_t M11;
extern const Coordinate_t M12;
extern const Coordinate_t M13;
extern const Coordinate_t M14;
extern const Coordinate_t M15;

/* Hoekplaat linksboven (moeren). */
extern const Coordinate_t LB00;
extern const Coordinate_t LB01;
extern const Coordinate_t LB02;
extern const Coordinate_t LB03;
extern const Coordinate_t LB04;
extern const Coordinate_t LB05;

/* RM-posities uit het coordinatenblad. */
extern const Coordinate_t RM00;
extern const Coordinate_t RM01;
extern const Coordinate_t RM02;
extern const Coordinate_t RM03;
extern const Coordinate_t RM04;
extern const Coordinate_t RM05;

/* Hoekplaat linksonder (moeren). */
extern const Coordinate_t LO00;
extern const Coordinate_t LO01;
extern const Coordinate_t LO02;
extern const Coordinate_t LO03;
extern const Coordinate_t LO04;
extern const Coordinate_t LO05;

#endif /* COORDINATES_H_ */
