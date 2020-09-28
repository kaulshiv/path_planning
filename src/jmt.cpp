#include <iostream>
#include <string>
#include "jmt.h"


void JMT::set_coeffs(const vector<double> &start, const vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  MatrixXd A(3,3);
  VectorXd b(3), ans(3);
  double si=start[0], si_d=start[1], si_dd=start[2];
  double sf=end[0], sf_d=end[1], sf_dd=end[2];

  JMT::coeffs = {si, si_d, 0.5*si_dd, 0, 0, 0};
   
  A << pow(T,3), pow(T,4), pow(T,5),
        3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
        6*T, 12*pow(T,2), 20*pow(T,3);
  b << sf - (si + si_d*T + 0.5*si_dd*pow(T,2)), sf_d - (si_d + si_dd*T), sf_dd - si_dd;
   
  ans << A.inverse()*b;
  JMT::coeffs[3] = ans[0];
  JMT::coeffs[4] = ans[1];
  JMT::coeffs[5] = ans[2];

}

double JMT::position(double x) const
{
  double ans = 0;
  for (int i=0; i<JMT::coeffs.size(); i++)
    ans += JMT::coeffs[i] * pow(x,i);

  return ans;
}

double JMT::velocity(double x) const
{
  return JMT::coeffs[1] + 2*JMT::coeffs[2]*x + 3*JMT::coeffs[3]*pow(x,2) + 4*JMT::coeffs[4]*pow(x,3) + 5*JMT::coeffs[5]*pow(x,4);
}


double JMT::acceleration(double x) const
{
  return 2*JMT::coeffs[2] + 6*JMT::coeffs[3]*x + 12*JMT::coeffs[4]*pow(x,2) + 20*JMT::coeffs[5]*pow(x,3);
}