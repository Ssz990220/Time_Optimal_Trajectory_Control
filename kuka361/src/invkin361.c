/***************************************************************************
  tag: Diederik Verscheure  di mrt 14 17:10:16 CET 2006  invkin361.c

                           invkin361.c -  description
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
  Returns the joint angles for a given end effector frame for the kuka361.
 ***************************************************************************/

/*
** NAME: InvPosZXXDWH.c
** CREATED: 12 nov 1992
** AUTHOR: Herman Bruyninckx, PMA, KU Leuven, Belgium
**
** FUNCTION: Calculation of the inverse POSITION kinematics of
**  a ZXXDWH robot,
**  as a particular case of a 6 rotational dof robot with no
**  offsets or eccentricities. The kinematical definitions are as
**  described in the references, with the exception that this
**  robot has a more complicated spherical wrist. 
**  *********************************************************
**  ***   Only the input configuration is calculated.     ***
**  *********************************************************
**
** ALGORITHM: The inverse kinematics of an `equivalent' ZXXZXZ robot is first
**  calculated. The equivalent robot has a spherical wrist, with
**  orthogonal axes.
**  Then, the fifth joint angle is determined, together with a
**  correction term for the fourth and sixth joint angles,
**  as described in Willems and Asselberghs (eqs.(3.1)-(3.8)).
**
** EXPORTS: int InvPosZXXDWH()
**
** INPUT: `p_d': pointer to ``Kine6RData'' struct; used fields:
**  `status':desired configuration.
**  `t6': the homogeneous transformation matrix of the end
**   effector wrt the base.
** RETURNS: `0': if desired output can be calculated;
**  `1': otherwise.
** OUTPUT: `status':possible singularities.
**  `q': joint angles.
** 
** CAVEATS: Joint angles are THEORETICAL, i.e., NO range checking is done!!
**
**  Angles are in radians; lengths are in millimeter.
**
**  At singularities we made the choice to let ``InvPosZXXDWH()''
**  return the OLD joint values, i.e. these given TO the function
**  (or whatever is contained in `q' on entry!).
** 
** REFERENCES: - Low and Dubey, Int.J.Rob.Research,Vol. 5, No. 4, 1986
**  - Brochier en Haverhals, Studie voor on-line toepassing van
**    inverse kinematika en dynamica, Thesis KULeuven, PMA, 1988.
**  - Willems and Asselberghs, Ontwerp en evaluatie van
**    uitwendige krachtcontrolemethoden voor industriele robots,
**    Thesis 92 EP 22, KULeuven, PMA, 1992.
**  - "feathZXXDWH.doc"
**  - "Kine6R.h"
*/



#include "math.h"
#include "mex.h" 

#define FSIGN(a) ( a<-1E-15 ? -1.0 : ( a>1E-15 ? 1.0 : 0.0))
#define SQRT3d2 (double)(0.8660254037844386) /* sqrt(3)/2 */

#define CONFIG_BACKWARD 1.0
#define CONFIG_ELBOWDOWN 2.0
#define CONFIG_FLIP 3.0
#define M_PI 3.1415926535897932
#define SING_ARMEXTENDED 1.0
#define SING_OUTOFREACH 2.0
#define SING_WRISTABOVEBASE 3.0
#define SING_PARALLELWRIST 4.0
#define SING_OTHER 5.0

double *q;
double *config;
double *frame;
double *q_eq; 

/* Link lengths
 */
double l1 = 1.020;
double l2 = 0.480;
double l3 = 0.645;
double l4 = 0.0;
double l5 = 0.0;
double l6 = 0.120;
double const1, const2, max_wrist_dist;
const double EPS_WRIST_ABOVE_BASE = 0.001; 
/* units: m
* decision variable: XY distance between wrist and shoulder */

const double EPS_ARM_EXTENDED = 0.001; 
/* units: m
 * decision variable: distance of wrist to shoulder must lie between
 *        sqrt(sq(L2+L3)) and sqrt(sq(L2-L3)). */

const double EPS_PARALLEL_WRIST = 0.001; 
/* units: dimensionless
 * decision variable: sine of flip angle (fifth joint) goes to zero. */

const double KINEMATICS_EPS = 1.0e-8;
/* Considered VERY small in the context of kinematic calculations. */
const double M_PI_T2 = 2.0 * M_PI;


double P6x, P6y, P6z; /* Coordinates (in the base frame) of the 6th link */
double Rxy_2;  /* square of XY distance betw. wrist & base of link2 */
double Rxyz_2;  /* square of XYZ distance betw. wrist & base of link2 */
double s1, c1, s2, c2, s3, c3, s5, c5; /* sin,cos of thetai */
double s23, c23, s1c23, c1c23; /* temporary... */
double d13, d23;
double temp1, temp2;   /* ...storage */
/* cosines and sines of `equivalent' robot: */
double c4_eq, s4_eq, c5_eq, s5_eq, c6_eq, s6_eq;
double alpha;  /* compensation term for joints 4 and 6 */
double Pwx, Pwy, Pwz;
double dWv, dWh;
double s;
            
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if ((nrhs < 2) || (nrhs > 2))
	{
		printf("Expecting 2 input arguments (frame, configuration)...\n");
		printf("Bailing out...\n");
		return;
	}
	else
	{

		if ((mxGetM(prhs[0]) != 4) || (mxGetN(prhs[0]) != 4))
		{
			printf("Expecting input argument 1 of size 4x4...\n");
			printf("Bailing out...\n");
			plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
			q = mxGetPr(plhs[0]);
			q[0] = -1.0;
			return;
		}
		else
		{
			frame = mxGetPr(prhs[0]);
		}
		if ((mxGetM(prhs[1]) != 1) || (mxGetN(prhs[1]) != 1))
		{
			printf("Expecting input argument 1 of size 1x1...\n");
			printf("Bailing out...\n");
			plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
			q = mxGetPr(plhs[0]);
			q[0] = -1.0;
			return;
		}
		else
		{
			config = mxGetPr(prhs[1]);


			plhs[0] = mxCreateDoubleMatrix(1, 6, mxREAL);
			q = mxGetPr(plhs[0]);
            q_eq = mxGetPr(mxCreateDoubleMatrix(1, 6, mxREAL));  /*Joint values of `equivalent' robot */
            const1 = l2*l2 + l3*l3;
            const2 = 2. * l2 * l3;
            max_wrist_dist = const1 + const2;
			
			s = 0.0;
			
			/* Coordinates of last link wrt base: */
			P6x = frame[0+2*4] * l6;
			P6y = frame[1+2*4] * l6;
			P6z = frame[2+2*4] * l6;
			
			/* Wrist position: (Low & Dubey eq.(34)) */
			Pwx = frame[0+3*4] - P6x;
			Pwy = frame[1+3*4] - P6y;
			Pwz = frame[2+3*4] - P6z;
			
			/* Calculation of relevant lengths: */
			dWv = Pwz - l1;
			Rxy_2 = Pwx * Pwx + Pwy * Pwy;
			Rxyz_2 = Rxy_2 + dWv * dWv;
			
			/* By Peter Soetens */
			
			if ( Rxy_2 < 0.0 )
			{
				plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
				q = mxGetPr(plhs[0]);
				s = SING_OTHER;
				q[0] = s;
			    	return;
			}
			
			if (config[0] == CONFIG_BACKWARD)
			    dWh = -sqrt( Rxy_2 );
			else
			    dWh = sqrt( Rxy_2 );
			
			/*  Arm extended? */
			if ( Rxyz_2 > ( max_wrist_dist - EPS_ARM_EXTENDED * EPS_ARM_EXTENDED ) )
			{
			    s = SING_ARMEXTENDED;
			    /* Out of reach? */
			
			    if ( Rxyz_2 > max_wrist_dist )
			        s = SING_OUTOFREACH;
			
			    /*    rtos_printf("ARM ERROR");*/
				plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
				q = mxGetPr(plhs[0]);
				q[0] = s;
				return;

			}
			
			/* Wrist is above shoulder? */
			if ( fabs( dWh ) < EPS_WRIST_ABOVE_BASE )
			{
				s = SING_WRISTABOVEBASE;
				/*    rtos_printf("WRIST ERROR");*/
				plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
				q = mxGetPr(plhs[0]);
				q[0] = s;
				return;
			}
			
			/* Calculation of q[0] by geometry of triangle formed by dWh and Rxy_2: */
			s1 = -Pwx * dWh / Rxy_2;
			
			c1 = Pwy * dWh / Rxy_2;
			
			q[ 0 ] = atan2( s1, c1 );
			
			/* Calculation of q[2]: (formulas (B6) and (36), Low and Dubey) */
			c3 = ( Rxyz_2 - const1 ) / const2;
			
			if ( 1.0 - c3 * c3 < 0.0 )
			{
				plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
				q = mxGetPr(plhs[0]);
				s = SING_OTHER;
				q[0] = s;
			    	return;
			}
			
			if (config[0] == CONFIG_ELBOWDOWN)
			    s3 = -sqrt ( 1.0 - c3 * c3 );
			else
			    s3 = sqrt( 1.0 - c3 * c3 );
			
			q[ 2 ] = atan2 ( s3, c3 );
			
			/* Calculation of q[1]: (formulas (37), Low and Dubey) */
			temp1 = l3 * c3 + l2;
			
			temp2 = l3 * s3;
			
			c2 = ( dWv * temp1 + dWh * temp2 ) / Rxyz_2;
			
			s2 = ( dWh - c2 * temp2 ) / temp1;
			
			q[ 1 ] = atan2( s2, c2 );
			
			/* Orientation of wrist wrt arm: Dij (formulas (42), Low and Dubey) */
			s23 = s2 * c3 + c2 * s3;
			
			c23 = c2 * c3 - s2 * s3;
			
			s1c23 = s1 * c23;
			
			c1c23 = c1 * c23;
			
			d13 = frame[0+2*4] * c1 + frame[1+2*4] * s1;
			
			d23 = -frame[0+2*4] * s1c23 + frame[1+2*4] * c1c23 - frame[2+2*4] * s23;
			
			/* Calculation of theta4, theta5 and theta6 of `equivalent' robot:
			   (formulas (39),(40),(41), Low and Dubey) */
			if ( ( fabs( d13 ) < KINEMATICS_EPS ) && ( fabs( d23 ) < KINEMATICS_EPS ) )
			{
			    /* Z-axes of links 4 and 6 are parallel! q[3] keeps its previous value! */
			    s4_eq = sin( q[ 3 ] );
			    c4_eq = cos( q[ 3 ] );
			    q_eq[ 4 ] = 0.0;
			    s5_eq = 0;
			}
			
			else
			{
			    if (config[0] == CONFIG_FLIP)
			        q_eq[ 3 ] = atan2 ( d13, -d23 );
			    else
			        q_eq[ 3 ] = atan2 ( -d13, d23 );
			
			    c4_eq = cos ( q_eq[ 3 ] );
			
			    if ( fabs( d23 ) > KINEMATICS_EPS )
			        s4_eq = - c4_eq * d13 / d23;
			    else
			        s4_eq = sin ( q_eq[ 3 ] );
			
			    c5_eq = -frame[0+2*4] * s1 * s23 + frame[1+2*4] * c1 * s23 + frame[2+2*4] * c23;
			
			    s5_eq = d23 * c4_eq - d13 * s4_eq;
			
			    q_eq[ 4 ] = atan2 ( s5_eq, c5_eq );
			}
			
			if ( fabs( s5_eq ) < EPS_PARALLEL_WRIST )
			    s = SING_PARALLELWRIST;
			
			s6_eq = -( frame[0+1*4] * c1 + frame[1+1*4] * s1 ) * c4_eq + ( frame[0+1*4] * s1c23 - frame[1+4*1] * c1c23 + frame[2+4*1] * s23 ) * s4_eq;
			
			c6_eq = ( frame[0+4*0] * c1 + frame[1+4*0] * s1 ) * c4_eq + ( -frame[0+4*0] * s1c23 + frame[1+4*0] * c1c23 - frame[2+4*0] * s23 ) * s4_eq;
			
			q_eq[ 5 ] = atan2 ( s6_eq, c6_eq );
			
			/* This far, the `equivalent' robot's inverse kinematics have been
			   calculated. Now, we determine the correction factor `alpha'
			   for the fourth and sixth joint positions.
			   (eqs. (3.1) -(3.8), Willems and Asselberghs). */
			
			/* remark : abs(q_eq[4]) is always smaller than 120 degrees */
			/* XXX Except when we have rounding errors ! (which happens) */
			
			c5 = 4. * cos ( q_eq[ 4 ] / 2. ) - 3.;
			if (c5 > 1.) c5 = 1; else if (c5 <-1.) c5=-1.; /* correction for rounding errors*/
			
			if ( q_eq[ 4 ] < -KINEMATICS_EPS )
			{
			    q[ 4 ] = -acos( c5 );
			    s5 = sin( q[ 4 ] );
			    alpha = atan2 ( -s5, SQRT3d2 * ( c5 - 1. ) );
			}
			
			else
			{
			    if ( q_eq[ 4 ] < KINEMATICS_EPS )
			    {
			        alpha = M_PI/2;
			        q[ 4 ] = 0.0;
			    }
			
			    else
			    {
			        q[ 4 ] = acos( c5 );
			        s5 = sin( q[ 4 ] );
			        alpha = atan2 ( s5, SQRT3d2 * ( 1. - c5 ) );
			    }
			}
			
			q[ 3 ] = q_eq[ 3 ] - alpha;
			
			if ( q[ 3 ] >= M_PI )
			    q[ 3 ] -= M_PI_T2;
			else if ( q[ 3 ] <= -M_PI )
			    q[ 3 ] += M_PI_T2;
			
			q[ 5 ] = q_eq[ 5 ] + alpha;
			
			if ( q[ 5 ] >= M_PI )
			    q[ 5 ] -= M_PI_T2;
			else if ( q[ 5 ] <= -M_PI )
			    q[ 5 ] += M_PI_T2;
			
			if(s > 0.0)
			{
				plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
				q = mxGetPr(plhs[0]);
				q[0] = s;
			}
			else
			{
				/* Correction for ZYYDWH */
				q[ 0 ] = - q[ 0 ] - M_PI / 2.0;
				q[ 3 ] = - q[ 3 ];
				q[ 5 ] = - q[ 5 ] + M_PI / 2.0;
			}

			
		}
	}
	return;
}




