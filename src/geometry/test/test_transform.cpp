/**
  * @file     	sensor.c
  * @author   	JonesLee
  * @email   	Jones_Lee3@163.com
  * @version	V4.01
  * @date    	07-DEC-2017
  * @license  	GNU General Public License (GPL)  
  * @brief   	Universal Synchronous/Asynchronous Receiver/Transmitter 
  * @detail		detail
  * @attention
  *  This file is part of OST.                                                  \n                                                                  
  *  This program is free software; you can redistribute it and/or modify 		\n     
  *  it under the terms of the GNU General Public License version 3 as 		    \n   
  *  published by the Free Software Foundation.                               	\n 
  *  You should have received a copy of the GNU General Public License   		\n      
  *  along with OST. If not, see <http://www.gnu.org/licenses/>.       			\n  
  *  Unless required by applicable law or agreed to in writing, software       	\n
  *  distributed under the License is distributed on an "AS IS" BASIS,         	\n
  *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  	\n
  *  See the License for the specific language governing permissions and     	\n  
  *  limitations under the License.
  *
  * @htmlonly 
  * <span style="font-weight: bold">History</span> 
  * @endhtmlonly 
  * Version|Auther|Date|Describe
  * ------|----|------|-------- 
  * V3.3|Jones Lee|07-DEC-2017|Create File
  * <h2><center>&copy;COPYRIGHT 2017 WELLCASA All Rights Reserved.</center></h2>
  */
#include <gtest/gtest.h>

#include "geometry/transform.h"
#include "user_interface/user_interface.h"

#include <complex>

Eigen::MatrixXd randMat = get_random_matrix(3,12);
std::vector<double> randNum = get_uniform_double();

Eigen::Vector3d p1 = randMat.col(0), p2 = randMat.col(1), p3 = randMat.col(2),
                p4 = randMat.col(3), k1 = randMat.col(4).normalized(),
                k2 = randMat.col(5).normalized(),
                k3 = randMat.col(6).normalized(),
                k4 = randMat.col(7).normalized(),
                h1 = randMat.col(8).normalized(),
                h2 = randMat.col(9).normalized(),
                h3 = randMat.col(10).normalized(),
                h4 = randMat.col(11).normalized();

double q1 = randNum[0] * 2 * M_PI - M_PI, q2 = randNum[1] * 2 * M_PI - M_PI,
       q3 = randNum[2] * 2 * M_PI - M_PI, q4 = randNum[3] * 2 * M_PI - M_PI;
double d1, d2;

// Subproblem 4: Circle and plane
TEST(CanonicalSubproblem, sub4) {
  d1 = h1.transpose()*lieSO3(k1*q1)*p1;

  std::vector<double> ans = canonical_subproblem_4(p1, k1, h1.normalized(), d1);

  std::cout << "Total: " << ans.size() << " sulutions:" << std::endl;
  for (auto& x : ans) {
    std::cout << "rad = " << x << ", deg = " << x*180/M_PI << std::endl;
    ASSERT_TRUE(verify_canonical_subproblem_4(p1,k1,h1,d1,x));
  }
}

// Subproblem 1: Circle and point
TEST(CanonicalSubproblem, sub1) {
  p2 = lieSO3(k1*q1)*p1;

  std::cout << "p1 = " << p1.transpose() << "; k = " << k1.transpose() <<"; q = " << q1 << std::endl;
  std::cout << "p2 = " << p2.transpose() << std::endl;

  double ans = canonical_subproblem_1(p1, k1, p2);
  std::cout << "q = " << ans << std::endl;
  ASSERT_TRUE(verify_canonical_subproblem_1(p1, k1, p2, ans));
}

// Subproblem 2: Two circles
TEST(CanonicalSubproblem, sub2) {
  p2 = lieSO3(-k2*q2)*(lieSO3(k1*q1)*p1);

  std::cout << "p1 = " << p1.transpose() << "; k1 = " << k1.transpose() <<"; q1 = " << q1 << std::endl;
  std::cout << "p2 = " << p2.transpose() << "; k2 = " << k2.transpose() <<"; q2 = " << q2 << std::endl;

  std::vector<std::vector<double>> ans = canonical_subproblem_2(p1, k1, p2, k2);
  for (auto& q : ans) {
    std::cout << "q: " << q[0] << ", " << q[1] << std::endl;
    ASSERT_TRUE(verify_canonical_subproblem_2(p1, k1, p2, k2, q[0], q[1]));
  }
}

// Subproblem 3: Circle and sphere
TEST(CanonicalSubproblem, sub3) {
  d1 = (lieSO3(k1*q1)*p1 - p2).norm();

  std::cout << "p1 = " << p1.transpose() << "; k = " << k1.transpose() << "; q = " << q1 << std::endl;
  std::cout << "p2 = " << p2.transpose() << "; d = " << d1 << std::endl;

  std::vector<double> ans = canonical_subproblem_3(p1, k1, p2, d1);
  std::cout << "Total: " << ans.size() << " sulutions:" << std::endl;
  for (auto& theta : ans) {
    std::cout << "q = " << theta << std::endl;
    ASSERT_TRUE(verify_canonical_subproblem_3(p1, k1, p2, d1, theta));
  }
}

// Subproble 6: Four circles
// TEST(CanonicalSubproblem, sub6) {
//   canonical_subproblem_6(p1,k1,h1,p2,k2,h2,d1,p3,k3,h3,p4,k4,h4,d2);
// }

TEST(SolveEllipse, isEllipse) {
  // 生成随机椭圆
  // Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  // Eigen::Matrix2d A = (lieSO3(Eigen::Vector3d(0,0,1)*(randNum[0]*2*M_PI-M_PI))*I).block<2,2>(0,0);
  // Eigen::Matrix2d M = A.col(0)*A.col(0).transpose()/randNum[1]/randNum[1] + A.col(1)*A.col(1).transpose()/randNum[2]/randNum[2];
  Eigen::Matrix2d A = get_random_matrix(2,2);
  Eigen::Matrix2d M = A.transpose()*A;
  // 椭圆中心点
  Eigen::Vector2d C = get_random_matrix(2,1);

  std::vector<double> k(5,0);
  k[0] = C.transpose()*M*C - 1;
  k[1] = -2*((C.transpose()*M)(0));
  k[2] = -2*((C.transpose()*M)(1));
  k[3] = M(0,0);
  k[4] = M(1,0) + M(0,1);
  for (size_t i=0; i<k.size(); ++i) {
    k[i] /= M(1,1);
  }

  std::cout << "M = \n" << M << std::endl;
  ASSERT_TRUE(is_ellipse(k));
}

TEST(SolveEllipse, solve_cubic_equation) {
  std::vector<double> k = get_uniform_double(4);
  // k = {-4, -15, 0, 1};
  auto ans = solve_cubic_equation(k);
  std::cout << "k = " << k[0] << ", " << k[1] << ", " << k[2] << ", " << k[3] << std::endl;
  for (auto sol : ans) {
    double x = sol.first;
    double fx = k[0] + k[1]*x + k[2]*x*x + k[3]*x*x*x;
    std::cout << "x = " << sol.first << "(" << sol.second << "), fx = " << fx << std::endl;
    ASSERT_TRUE(fabs(fx) < 1e-12);
  }
}

TEST(SolveEllipse, solve_quartic_equation) {
  std::vector<double> k = get_uniform_double(5);
  std::map<double, int> ans = solve_quartic_equation(k);
  std::cout << "k = " << k[0] << ", " << k[1] << ", " << k[2] << ", " << k[3] << ", " << k[4] << std::endl;
  std::cout << "ans.size() = " << ans.size() << std::endl;
  for (auto& sol : ans) {
    double x = sol.first;
    double fx = k[0] + k[1]*x + k[2]*x*x + k[3]*x*x*x + k[4]*x*x*x*x;
    std::cout << "x = " << x << ", fx = " << fx << std::endl;
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

