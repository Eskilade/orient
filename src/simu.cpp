#include <gtsam/geometry/Rot3.h>
#include <iostream>
#include <normal_multivariate.hpp>
#include <fstream>

const static Eigen::IOFormat CSV(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", " ");

using namespace gtsam;

template<std::size_t N, typename F>
void do_(F f)
{
  for(auto i =0u; i<N;++i)
    f();
}

int main()
{
  Vector3 tau = Vector3::Random();
  auto X0 = Rot3::Expmap(tau);

  Matrix3 cov = Matrix3::Zero();
  cov.diagonal() << 1e-2,2e-3,1e-3;
  cov(0,1) = 1e-3;
  cov(0,2) = -1e-3;
  cov(1,2) = 5e-4;


  auto cov2 = 0.5 * (cov + cov.transpose());
  normal_multivariate dist(cov2);

  std::vector<Vector3> samples{};
  std::ofstream f{"/tmp/test.txt"};

  do_<1000>([&]{
    auto s = Rot3::Logmap(X0 * Rot3::Expmap(dist()));
    samples.push_back(s);
    f << s.format(CSV) << "\n";
  });

  Vector3 avg = Vector3::Zero();
  for(auto const& s  : samples)
    avg += s;
  avg /= samples.size();

  std::cout << tau.format(CSV) << "\n";
  std::cout << avg.format(CSV) << "\n";
	return 0;
}
