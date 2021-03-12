/*
 * @file	: screen.h
 *
 *  imported on : 09-mar-2020.
 *  Modified on : 27-oct-2019. Redefinition des codes VT100
 *  Created on: 03 fev. 2019
 *      Author: Jean
 */

#ifndef VT100_H_
#define VT100_H_

/*
 * ---- codes VT100----
 * \x1B = ESC
 * ESC[2J           efface ecran
 * ESC[H            reset curseur
 * ESC[?25l         curseur invisible
 * ESC[?25h         curseur visible
 * Esc[Line;ColH    position curseur
 */

/* Valid ANSI Mode Control Sequences for VT100 */
/*  macro avec quote
 *  #define macro(sequence) "\"" #sequence "\""
 */
#define CPR(l,c)	"\x1B[l;cR"		//Cursor Position Report -- VT100 to Host
#define CUB(c)		"\x1B[cD"		//Cursor Backward -- Host to VT100 and VT100 to Host
#define CUD(l)		"\x1B[lB"		//Cursor Down -- Host to VT100 and VT100 to Host
#define CUF(c)		"\x1B["#c"C"			//Cursor Forward -- Host to VT100 and VT100 to Host
#define CUP(r,c)	"\x1B[" #r ";" #c "H"	//Cursor Position (ligne,colonne)
#define GOTO(r,c)	"\x1B[r;cH"				//Cursor position -- erreur de preprossesor
#define CUU(l)		"\x1B[lA"		//Cursor Up --Host to VT100 and VT100 to Host

#define clrscr		"\x1B[2J"
#define ERASELINE	"\x1B[0K"		//Erase In Line -- efface la ligne
#define homescr		"\x1B[H"

/* CHAR MODES */
#define OFF			"\x1B[0m"
#define BOLD		"\x1B[1m"	//et Jaune!
#define LOWINTENS	"\x1B[2m"
#define UNDERLINE	"\x1B[4m"
#define BLINK		"\x1B[5m"
#define REVERSE		"\x1B[7m"

#define DECSC		"\x1B" "7"		//Sauve cursor
#define DECRC		"\x1B" "8"		//Restore cursor

#endif /* VT100_H_ */
