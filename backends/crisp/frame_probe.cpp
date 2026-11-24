// Original API client: a rotated, frictionless contact disambiguates frames.
#include <crisp/crisp.hpp>
#include <iostream>
#include <iomanip>

int main() {
  auto b=crisp::make_model_builder();b->disableGround();b->opt().dt=.001;b->opt().gravity={0.,0.,-9.81};
  b->opt().sol.type=crisp::solver_e::canal;b->opt().sol.min_res=1e-10;
  Eigen::Quaterniond Q=Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitY());
  Eigen::Vector3d n=Q*Eigen::Vector3d::UnitZ();
  auto& ramp=b->addBody({.name="ramp",.pos=-.025*n,.quat=Q,.mass=1.,.inertia={1.,1.,1.,0.,0.,0.}},true);
  ramp.addGeom({.name="ramp_geom",.type=crisp::geometry_e::box,.size=crisp::make_box_size(1.,1.,.05),.mu=0.});
  auto& sphere=b->addBody({.name="sphere",.pos=.019999*n,.mass=.1,.inertia={.000016,.000016,.000016,0.,0.,0.}},false);
  sphere.addGeom({.name="sphere_geom",.type=crisp::geometry_e::sphere,.size=crisp::make_sphere_size(.02),.mu=0.});
  auto m=b->build();if(!m)return 2;auto d=crisp::make_data(*m);crisp::reset(*m,*d);
  const auto array=[](const auto& v){std::cout<<'[';for(int i=0;i<v.size();++i){if(i)std::cout<<',';std::cout<<v[i];}std::cout<<']';};
  std::cout<<std::setprecision(17);
  for(int k=0;k<10;++k) {
    crisp::step(*m,*d);
    if(d->con.lambda.size() && d->con.lambda[0].norm()>1e-12) {
      const auto& c=d->con.feature[0];Eigen::Vector3d a=-d->con.rot[0]*d->con.lambda[0]/.001,bf=-d->con.rot[0].transpose()*d->con.lambda[0]/.001;
      Eigen::Vector3d rcol=d->con.rot[0].col(2),rrow=d->con.rot[0].row(2).transpose();
      const double sign=m->geom.type[c.geom1]==crisp::geometry_e::sphere ? 1. : -1.;
      Eigen::Vector3d on_sphere=-sign*bf;
      Eigen::Vector3d expected=.1*d->state.v().head<3>()/.001-Eigen::Vector3d(0.,0.,-.981);
      const double balance=(on_sphere-expected).norm();
      std::cout<<"{\"normal\":";array(c.normal);std::cout<<",\"lambda\":";array(d->con.lambda[0]);
      std::cout<<",\"R_col2\":";array(rcol);std::cout<<",\"R_row2\":";array(rrow);
      std::cout<<",\"negative_R_lambda_over_h\":";array(a);std::cout<<",\"negative_RT_lambda_over_h\":";array(bf);
      std::cout<<",\"force_on_sphere_N\":";array(on_sphere);std::cout<<",\"independent_impulse_balance_N\":";array(expected);
      std::cout<<",\"linear_momentum_residual_N\":"<<balance<<",\"passed\":"
        <<(balance<1e-4 && (rrow-c.normal).norm()<1e-10 ? "true" : "false")<<"}\n";
      return 0;
    }
  }
  return 3;
}
