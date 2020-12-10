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

double *q;
double *M;
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

double t1, t2, t3, t4, t10, t11, t12, t13, t14, t15, t16, t17, t19, t20, t21, t36, t37, t38, t40, t45;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if ((nrhs < 1) || (nrhs > 1))
	{
		printf("Expecting 1 input argument (q)...\n");
		printf("Bailing out...\n");
		plhs[0] = mxCreateDoubleMatrix(6, 6, mxREAL);
		return;
	}
	else
	{
		if ((mxGetM(prhs[0]) != 1) || (mxGetN(prhs[0]) != 6))
		{
			printf("Expecting 1 input argument of size 1x6...\n");
			printf("Bailing out...\n");
			plhs[0] = mxCreateDoubleMatrix(6, 6, mxREAL);
			return;
		}
		else
		{
            q = mxGetPr(prhs[0]);

			plhs[0] = mxCreateDoubleMatrix(6, 6, mxREAL);
			M = mxGetPr(plhs[0]);


			/* Mass matrix
			 */ 
			t1 = cos(q[1]);
			t2 = t1 * t1;
			t3 = sin(q[1]);
			t4 = t3 * t3;
			t10 = sin(q[2]);
			t11 = D13 * t10;
			t12 = q[1] + q[2];
			t13 = sin(t12);
			t14 = t13 * t13;
			t15 = cos(t12);
			t16 = t15 * t15;
			t17 = t14 - t16;
			t19 = cos(q[2]);
			t20 = D13 * t19;
			t21 = t13 * t15;
			M[0+0*6] = y_ls[0] + (t2 - t4) * y_ls[1] - 0.2e1 * t1 * t3 * y_ls[2] + (t11 - t11 * t17 - 0.2e1 * t20 * t21) * y_ls[5] + (t20 - 0.2e1 * t11 * t21 + t20 * t17) * y_ls[6] + t17 * y_ls[7] + 0.2e1 * t21 * y_ls[8];
			t36 = t13 * y_ls[9];
			t37 = t15 * y_ls[10];
			M[0+1*6] = t1 * y_ls[3] - t3 * y_ls[4] + t36 + t37;
			M[0+2*6] = t36 + t37;
			M[1+0*6] = M[0+1*6];
			t38 = t11 * y_ls[5];
			t40 = t20 * y_ls[6];
			M[1+1*6] = 0.2e1 * t38 + 0.2e1 * t40 + y_ls[13] + y_ls[14] + y_ls[15] / 0.1000e4;
			M[1+2*6] = t38 + t40 + y_ls[14] + r3 * y_ls[15] / 0.1000e4;
			M[2+0*6] = M[0+2*6];
			M[2+1*6] = M[1+2*6];
			t45 = r3 * r3;
			M[2+2*6] = y_ls[14] + t45 * y_ls[15] / 0.1000e4;

			M[4+3*6] = 0.0;
			M[5+3*6] = 0.0;
			M[3+4*6] = 0.0;
			M[5+4*6] = 0.0;
			M[3+5*6] = 0.0;
			M[4+5*6] = 0.0;
			M[3+3*6] = y_ls2[0*3+0];
			M[4+4*6] = y_ls2[0*3+1];
			M[5+5*6] = y_ls2[0*3+2];


		}
	}
	return;
}



