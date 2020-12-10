/***************************************************************************
  tag: Diederik Verscheure  di mrt 14 17:10:16 CET 2006  eeframe.c

                           eeframe361.c -  description
                           ----------------------------
    begin                : di maart 14 2006
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
  Returns the end effector frame for the kuka361.
 ***************************************************************************/

#include "math.h"
#include "mex.h" 

#define FSIGN(a) ( a<-1E-15 ? -1.0 : ( a>1E-15 ? 1.0 : 0.0))
#define M_PI_2 3.1415926535897932/2
double *q;
double *qt;
double *mpb;
double *mpt;
double *mpo;
double *q_eq;
int i, j, k;

/* Link lengths
 */
double l1 = 1.020;
double l2 = 0.480;
double l3 = 0.645;
double l4 = 0.0;
double l5 = 0.0;
double l6 = 0.120;


double c5, s5, c5_eq, alpha, c1, s1, c23, s23, c4, s4, c6, s6, s5c6, s5s6, c4s5, s4s5, c1c23, c1s23, s1c23, s1s23;
double dWv, dWh, P6x, P6y, P6z, Pwx, Pwy, Pwz;
            
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if ((nrhs < 1) || (nrhs > 2))
	{
		printf("Expecting 1 or 2 input arguments (q, offset frame)...\n");
		printf("Bailing out...\n");
		return;
	}
	else
	{

		if (nrhs == 2)
		{
			if ((mxGetM(prhs[1]) != 4) || (mxGetN(prhs[1]) != 4))
			{
				printf("Expecting input argument 2 of size 4x4...\n");
				printf("Bailing out...\n");
				return;
			}
			else
			{
				mpo = mxGetPr(prhs[1]);
			}
		}
		else
		{
			mpo = mxGetPr(mxCreateDoubleMatrix(4, 4, mxREAL));
			for(i = 0; i < 4*4; i++)
			{
				mpo[i] = 0.0;
			}
			mpo[0*4+0] = 1.0;
			mpo[1*4+1] = 1.0;
			mpo[2*4+2] = 1.0;
			mpo[3*4+3] = 1.0;
		}
		if ((mxGetM(prhs[0]) != 1) || (mxGetN(prhs[0]) != 6))
		{
			printf("Expecting input argument 1 of size 1x6...\n");
			printf("Bailing out...\n");
			return;
		}
		else
		{
			qt = mxGetPr(prhs[0]);
			plhs[0] = mxCreateDoubleMatrix(4, 4, mxREAL);
			mpt = mxGetPr(plhs[0]);
			mpb = mxGetPr(mxCreateDoubleMatrix(4, 4, mxREAL));
			q_eq = mxGetPr(mxCreateDoubleMatrix(1, 6, mxREAL));
			q = mxGetPr(mxCreateDoubleMatrix(1, 6, mxREAL));

			q[0] = - qt[0] - M_PI_2;
			q[1] = qt[1];
			q[2] = qt[2];
			q[3] = - qt[3];
			q[4] = qt[4];
			q[5] = - qt[5] + M_PI_2;
			
			c5 = cos(q[4]);
			s5 = sin(q[4]);
			c5_eq = (c5 + 3.0)/4.0;
			
			alpha = 0.0;	
			if (q[4] < -1e-15)
			{
				alpha = atan2(-s5, 0.8660254037844386*(c5 - 1.0));
				q_eq[4] = -2.0*acos(c5_eq);
			}
			else
			{
				if (q[4] < 1e-15)
				{
					alpha = M_PI_2;
					q_eq[4] = 0.0;
				}
				else
				{
					alpha = atan2(s5, 0.8660254037844386*(1.0 - c5));
					q_eq[4] = 2.0*acos(c5_eq);
				}
			}
			
			q_eq[3] = q[3] + alpha;
			q_eq[5] = q[5] - alpha;
			
			
			c1 = cos(q[0]);
			s1 = sin(q[0]);
			c23 = cos(q[1] + q[2]);
			s23 = sin(q[1] + q[2]);
			c4 = cos(q_eq[3]);
			s4 = sin(q_eq[3]);
			c5 = cos(q_eq[4]);
			s5 = sin(q_eq[4]);
			c6 = cos(q_eq[5]);
			s6 = sin(q_eq[5]);
			s5c6 = s5*c6;
			s5s6 = s5*s6;
			c4s5 = c4*s5;
			s4s5 = s4*s5;
			c1c23 = c1*c23;
			c1s23 = c1*s23;
			s1c23 = s1*c23;
			s1s23 = s1*s23;
			
			mpb[0*4+1] = s4*c5*s6 - c4*c6;
			mpb[0*4+2] = c4*c5*s6 + s4*c6;
			mpb[0*4+0] = -s1c23*mpb[0*4+2] - c1*mpb[0*4+1] + s1s23*s5s6;
			mpb[0*4+1] = c1c23*mpb[0*4+2] - s1*mpb[0*4+1] - c1s23*s5s6;
			mpb[0*4+2] = - s23*mpb[0*4+2] - c23*s5s6;
			
			mpb[1*4+1] = s4*c5*c6 + c4*s6;
			mpb[1*4+2] = c4*c5*c6 - s4*s6;
			mpb[1*4+0] = -s1c23*mpb[1*4+2] - c1*mpb[1*4+1] + s1s23*s5c6;
			mpb[1*4+1] = c1c23*mpb[1*4+2] - s1*mpb[1*4+1] - c1s23*s5c6;
			mpb[1*4+2] = - s23*mpb[1*4+2] - c23*s5c6;
			
			mpb[2*4+0] = -s1c23*c4s5 - c1*s4s5 - s1s23*c5;
			mpb[2*4+1] = c1c23*c4s5 - s1*s4s5 + c1s23*c5;
			mpb[2*4+2] = - s23*c4s5 + c23*c5;
			
			dWv = cos(q[1])*l2 + c23*l3;
			dWh = sin(q[1])*l2 + s23*l3;
			P6x = mpb[2*4+0]*l6;
			P6y = mpb[2*4+1]*l6;
			P6z = mpb[2*4+2]*l6;
			Pwx = -s1*dWh;
			Pwy = c1*dWh;
			Pwz = l1 + dWv;
			
			/* End-effector position
			 */
			mpb[3*4+0] = P6x + Pwx;
			mpb[3*4+1] = P6y + Pwy;
			mpb[3*4+2] = P6z + Pwz;
			
			mpb[0*4+3] = 0.0;
			mpb[1*4+3] = 0.0;
			mpb[2*4+3] = 0.0;
			mpb[3*4+3] = 1.0;

			for(i = 0; i < 4; i++)
			{
				for(j = 0; j < 4; j++)
				{
					mpt[j*4+i] = 0.0;
					for(k = 0; k < 4; k++)
					{
						mpt[j*4+i] = mpt[j*4+i] + mpb[k*4+i] * mpo[j*4+k];
					}
				}
			}

		}
	}
	return;
}


