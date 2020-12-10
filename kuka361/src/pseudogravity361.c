/***************************************************************************
  tag: Diederik Verscheure  wo mrt 29 17:12:33 CEST 2006  mass361.c

                           mass361.c -  description
                           ----------------------------
    begin                : wo maart 29 2006
    copyright            : (C) 2006 K.U.Leuven
    email                : diederik <dot> verscheure <at> mech <dot> kuleuven <dot> be

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/***************************************************************************
  Purpose
  --------------------------------------------------------------------------
  Calculates the mass matrix for the Kuka 361.
  
 ***************************************************************************/

#include "math.h"
#include "mex.h" 

#define FSIGN(a) ( a<-1E-15 ? -1.0 : ( a>1E-15 ? 1.0 : 0.0))

#define qd1 0.0
#define qd2 0.0
#define qd3 0.0
#define qd4 0.0
#define qd5 0.0
#define qd6 0.0

double *q;
double *dq;
double *G;
double D13 = 0.48;
double g1 = 9.81;
/* Identified parameters
 * Based on work of Walter Verdonck
 *
 * double y_ls[28] = {27.725, -10.313, -0.015108, 1.4306, 0.19196, 0.25374, 2.9728, 2.2883, 0.41185, 0.02511, 0.17287, 371.11, -0.20695, 34.685, 5.189, 0.0010077, 3.6155, -7.7045, 16.592, -9.2535, 12.69, 7.9581, 34.177, 51.058, 13.738, -12.958, -6.9991, 0.60874};
 */

double y_ls[26] = {29.0130, -10.9085, -0.2629, 2.6403, -0.0421, 0.0509, 3.0221, 2.4426, 0.0531, -0.2577, -0.2627, 346.7819, -0.2651, 35.4666, 5.6321, 0.9935, 3.2192, 10.7823, 34.2604, 16.9902, 39.1594, 5.4501, 16.1392, -12.7243, -6.1094, 1.9590};

/* Terms for rotor inertia axis 3 (PhD CG p101-102)
 */
double r3 = 51.44118; 
double l = 0.488;
double r = 0.1;

/* Wrist dynamic model
 * Based on work of Chris Ganseman
 * Parameter array, rows: parameters, columns: for different axes
 */
double y_ls2[12] = {3.4102, 3.2555, 2.2401, 29.5178, 35.0201, 28.1370, 16.3857, 15.5666, 14.5318, -0.5734, -2.1976, 5.8708};

double t3, t4, t8, t9, t10, t14, t16, t17, t18, t21, t22, t27, t43, t44, t47;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if ((nrhs < 2) || (nrhs > 2))
	{
		printf("Expecting 2 input arguments (q, dq)...\n");
		printf("Bailing out...\n");
		plhs[0] = mxCreateDoubleMatrix(6, 1, mxREAL);
		return;
	}
	else
	{
		if ((mxGetM(prhs[0]) != 1) || (mxGetM(prhs[1]) != 1) || (mxGetN(prhs[0]) != 6) || (mxGetN(prhs[1]) != 6))
		{
			printf("Expecting 2 input arguments of size 1x6...\n");
			printf("Bailing out...\n");
			plhs[0] = mxCreateDoubleMatrix(6, 1, mxREAL);
			return;
		}
		else
		{
			q = mxGetPr(prhs[0]);
			dq = mxGetPr(prhs[1]);

			plhs[0] = mxCreateDoubleMatrix(6, 1, mxREAL);
			G = mxGetPr(plhs[0]);


			/* Pseudo gravity vector
			 */ 
			G[0+0*1] = y_ls[23] + qd1 * y_ls[17] + FSIGN(dq[0]) * y_ls[18];
			t3 = l * r;
			t4 = sin(q[1]);
			t8 = l * l;
			t9 = r * r;
			t10 = cos(q[1]);
			t14 = sqrt(t8 + t9 - 0.2e1 * t3 * t10);
			t16 = y_ls[5] * t14;
			t17 = g1 * t10;
			t18 = cos(q[2]);
			t21 = g1 * t4;
			t22 = sin(q[2]);
			t27 = y_ls[6] * t14;
			G[1+0*1] = -(0.1000e4 * t3 * t4 * y_ls[16] - y_ls[24] * t14 - t16 * t17 * t18 + t16 * t21 * t22 - t4 * y_ls[11] * t14 + t27 * t21 * t18 + t27 * t17 * t22 - FSIGN(dq[1]) * y_ls[20] * t14 - qd2 * y_ls[19] * t14 + t17 * y_ls[12] * t14) / t14;
			t43 = q[1] + q[2];
			t44 = cos(t43);
			t47 = sin(t43);
			G[2+0*1] = FSIGN(dq[2]) * y_ls[22] + y_ls[5] * g1 * t44 - y_ls[6] * g1 * t47 + qd3 * y_ls[21] + y_ls[25];
			G[3+0*1] = (y_ls2[1*3+0]*qd4 + y_ls2[2*3+0]*FSIGN(dq[3]) + y_ls2[3*3+0]);
			G[4+0*1] = (y_ls2[1*3+1]*qd5 + y_ls2[2*3+1]*FSIGN(dq[4]) + y_ls2[3*3+1]);
			G[5+0*1] = (y_ls2[1*3+2]*qd6 + y_ls2[2*3+2]*FSIGN(dq[5]) + y_ls2[3*3+2]);

			return;
		}
	}
}
