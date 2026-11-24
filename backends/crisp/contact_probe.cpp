// Original API client. CRISP (c) 2026 Seoul National University.
// Preserve the unmodified SDK's LICENSE and NOTICE; no SDK is redistributed.
#include <crisp/crisp.hpp>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  const std::string solver = argc > 1 ? argv[1] : "canal";
  const double dt = argc > 2 ? std::stod(argv[2]) : .001;
  if ((solver != "canal" && solver != "sub_admm") || !(dt > 0 && dt <= .005)) return 2;
  auto builder = crisp::make_model_builder();
  builder->enableGround();
  builder->opt().dt = dt;
  builder->opt().gravity = {0., 0., -9.81};
  builder->opt().sol.type = solver == "canal" ? crisp::solver_e::canal : crisp::solver_e::sub_admm;
  builder->opt().sol.min_res = 1e-9;
  builder->opt().canal.max_iter = 100;
  builder->opt().sub_admm.max_iter = 500;
  const double mass=.1, radius=.02, inertia=.4*mass*radius*radius;
  auto& body=builder->addBody({.name="probe",.pos={0.,0.,.2},.mass=mass,
                              .inertia={inertia,inertia,inertia,0.,0.,0.}},false);
  body.addGeom({.name="sphere",.type=crisp::geometry_e::sphere,
                .size=crisp::make_sphere_size(radius),.mu=.5});
  auto model=builder->build();
  if (!model) return 3;
  auto data=crisp::make_data(*model);
  crisp::reset(*model,*data);
  Eigen::Vector3d support=Eigen::Vector3d::Zero();
  double max_residual=0.;int samples=0, contact_samples=0;
  for(int i=0;i<std::lround(1./dt);++i) {
    crisp::step(*model,*data);
    if(!data->state.q().allFinite() || !data->state.v().allFinite())return 4;
    if(data->sim.time>.7) {
      ++samples;contact_samples+=data->size.ncon>0;max_residual=std::max(max_residual,data->sol.res);
      for(int k=0;k<data->con.lambda.size();++k) support += data->con.rot[k].transpose()*data->con.lambda[k];
    }
  }
  std::cout<<std::setprecision(15)<<"{\"engine\":\"crisp\",\"solver\":\""<<solver
    <<"\",\"dt\":"<<dt<<",\"time\":"<<data->sim.time
    <<",\"height_m\":"<<data->state.q()[2]<<",\"raw_support_world\":["
    <<support[0]/samples<<","<<support[1]/samples<<","<<support[2]/samples
    <<"],\"weight_N\":"<<mass*9.81<<",\"contact_fraction\":"<<double(contact_samples)/samples
    <<",\"maximum_solver_residual\":"<<max_residual;
  if(data->size.ncon) {
    const auto& feature=data->con.feature[0];
    std::cout<<",\"pair\":["<<feature.geom1<<","<<feature.geom2<<"],\"normal\":["
      <<feature.normal[0]<<","<<feature.normal[1]<<","<<feature.normal[2]<<"]";
  }
  std::cout<<"}\n";
  return 0;
}
