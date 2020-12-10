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
double *dq;
double *C;
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

double t1, t2, t4, t6, t7, t9, t11, t12, t13, t15, t16, t18, t23, t27, t28, t29, t30, t32, t33, t37, t40, t46, t47, t50, t53, t60, t63, t66;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if ((nrhs < 2) || (nrhs > 2))
	{
		printf("Expecting 2 input arguments (q, dq)...\n");
		printf("Bailing out...\n");
		plhs[0] = mxCreateDoubleMatrix(6, 6, mxREAL);
		return;
	}
	else
	{
		if ((mxGetM(prhs[0]) != 1) || (mxGetM(prhs[1]) != 1) || (mxGetN(prhs[0]) != 6) || (mxGetN(prhs[1]) != 6))
		{
			printf("Expecting 2 input arguments of size 1x6...\n");
			printf("Bailing out...\n");
			plhs[0] = mxCreateDoubleMatrix(6, 6, mxREAL);
			return;
		}
		else
		{
			q = mxGetPr(prhs[0]);
			dq = mxGetPr(prhs[1]);

			plhs[0] = mxCreateDoubleMatrix(6, 6, mxREAL);
			C = mxGetPr(plhs[0]);


			/* Pseudo Coriolis matrix
			 */ 
			t1 = cos(q[1]);
			t2 = sin(q[1]);
			t4 = t1 * t2 * y_ls[1];		
			t6 = t1 * t1;	
			t7 = t2 * t2;	
			t9 = (t6 - t7) * y_ls[2];
			t11 = D13 * t1;
			t12 = q[1] + q[2];
			t13 = cos(t12);
			t15 = D13 * t2;
			t16 = sin(t12);
			t18 = -t11 * t13 + t15 * t16;
			t23 = t11 * t16 + t15 * t13;
			t27 = t16 * t13 * y_ls[7];
			t28 = 0.4e1 * t27;
			t29 = t16 * t16;
			t30 = t13 * t13;
			t32 = (t29 - t30) * y_ls[8];
			t33 = 0.2e1 * t32;
			t37 = t15 * t16 * y_ls[5];
			t40 = t15 * t13 * y_ls[6];
			C[0+0*6] = (-0.4e1 * t4 - 0.2e1 * t9 + 0.2e1 * t18 * y_ls[5] + 0.2e1 * t23 * y_ls[6] + t28 - t33) * dq[1] + (0.2e1 * t37 + 0.2e1 * t40 + t28 - t33) * dq[2];
			t46 = t13 * y_ls[9];
			t47 = t16 * y_ls[10];
			t50 = t46 - t47;
			C[0+1*6] = (-t2 * y_ls[3] - t1 * y_ls[4] + t46 - t47) * dq[1] + 0.2e1 * t50 * dq[2];
			C[0+2*6] = t50 * dq[2];
			C[0+3*6] = 0.0;
			C[0+4*6] = 0.0;
			C[0+5*6] = 0.0;
			t53 = 0.2e1 * t27;
			C[1+0*6] = (-t53 + t32 + 0.2e1 * t4 + t9 - t18 * y_ls[5] - t23 * y_ls[6]) * dq[0];
			t60 = cos(q[2]);
			t63 = sin(q[2]);
			t66 = D13 * t60 * y_ls[5] - D13 * t63 * y_ls[6];
			C[1+1*6] = 0.2e1 * t66 * dq[2];
			C[1+2*6] = t66 * dq[2];
			C[1+3*6] = 0.0;
			C[1+4*6] = 0.0;
			C[1+5*6] = 0.0;
			C[2+0*6] = (-t53 + t32 - t37 - t40) * dq[0];
			C[2+1*6] = -t66 * dq[1];
			C[2+2*6] = 0.0;
			C[2+3*6] = 0.0;
			C[2+4*6] = 0.0;
			C[2+5*6] = 0.0;
			C[3+0*6] = 0.0;
			C[3+1*6] = 0.0;
			C[3+2*6] = 0.0;
			C[3+3*6] = 0.0;
			C[3+4*6] = 0.0;
			C[3+5*6] = 0.0;
			C[4+0*6] = 0.0;
			C[4+1*6] = 0.0;
			C[4+2*6] = 0.0;
			C[4+3*6] = 0.0;
			C[4+4*6] = 0.0;
			C[4+5*6] = 0.0;
			C[5+0*6] = 0.0;
			C[5+1*6] = 0.0;
			C[5+2*6] = 0.0;
			C[5+3*6] = 0.0;
			C[5+4*6] = 0.0;
			C[5+5*6] = 0.0;
			return;
		}
	}
}
