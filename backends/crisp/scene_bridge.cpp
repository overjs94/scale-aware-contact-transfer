// Original public-API client; the CRISP SDK is not redistributed or modified.
#include <crisp/crisp.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

// Manifest and command protocol are private to the Python adapter. All names
// are whitespace-free generated identifiers; stdout STATE lines contain JSON.
int main(int argc, char** argv) try {
  if (argc != 2) return 2;
  std::ifstream input(argv[1]);
  if (!input) throw std::runtime_error("Cannot open scene manifest");
  std::string solver; double dt, mu; int count;
  input >> solver >> dt >> mu >> count;
  if (!(dt > 0) || !(mu >= 0) || count < 1 ||
      (solver != "canal" && solver != "sub_admm")) return 2;
  auto b = crisp::make_model_builder(); b->enableGround();
  b->opt().dt=dt; b->opt().gravity={0.,0.,-9.81};
  b->opt().sol.type=solver=="canal" ? crisp::solver_e::canal : crisp::solver_e::sub_admm;
  b->opt().sol.min_res=1e-8; b->opt().canal.max_iter=200;
  b->opt().sub_admm.max_iter=1000;
  std::vector<std::string> paths(count);
  for (int i=0;i<count;++i) {
    int fixed; Eigen::Vector3d p; double x,y,z,w;
    input >> paths[i] >> fixed >> p[0] >> p[1] >> p[2] >> x >> y >> z >> w;
    if (!b->addRobot({.path=paths[i],.root_pos=p,.root_quat={w,x,y,z}},fixed))
      throw std::runtime_error("Cannot import "+paths[i]);
  }
  int n; input >> n; std::vector<std::string> joints(n); std::vector<double> initial(n);
  for (int i=0;i<n;++i) {
    input >> joints[i] >> initial[i];
    if (!b->addActuator(joints[i],{.type=crisp::actuator_e::force,.gain={1.,0.,0.}}))
      throw std::runtime_error("Cannot actuate "+joints[i]);
  }
  int nex;input>>nex;std::vector<std::string> excludes(2*nex);
  for (int i=0;i<nex;++i) {
    input>>excludes[2*i]>>excludes[2*i+1];
    if (!b->addExclude(excludes[2*i],excludes[2*i+1]))
      throw std::runtime_error("Invalid collision exclusion");
  }
  std::string object;input>>object;
  if (!input) throw std::runtime_error("Incomplete manifest");
  auto m=b->build(); if(!m) throw std::runtime_error("CRISP model build failed");
  auto d=crisp::make_data(*m);crisp::reset(*m,*d);
  std::vector<int> qa(n,-1),va(n,-1),actuator(n,-1); int ob=-1;
  for(int j=0;j<m->size.nbody;++j) {
    if (m->body.nameid[j]>=0 && m->str.names[m->body.nameid[j]].str()==object) ob=j;
    if(m->jnt.nameid[j]<0)continue;
    auto name=m->str.names[m->jnt.nameid[j]].str();
    for(int i=0;i<n;++i)if(name==joints[i]) {qa[i]=m->jnt.qadr[j];va[i]=m->jnt.vadr[j];}
  }
  if(ob<0 || m->size.nu!=n) throw std::runtime_error("Object/actuator mapping failed");
  for(int a=0;a<m->size.nu;++a) {
    const auto name=m->str.names[m->jnt.nameid[m->act.jointid[a]]].str();
    for(int i=0;i<n;++i)if(name==joints[i])actuator[i]=a;
  }
  for(int i=0;i<n;++i) {
    if(qa[i]<0 || va[i]<0 || actuator[i]<0)throw std::runtime_error("Joint mapping failed");
    d->state.q()[qa[i]]=initial[i];
  }
  // The URDF importer names mesh assets by basename and may reuse those names
  // across arms. Native geometry IDs + owning body give unambiguous aliases.
  std::vector<std::string> geom_names(m->size.ngeom);std::vector<int> geometry_count(m->size.nbody,0);
  for(int g=0;g<m->size.ngeom;++g) {
    const int body=m->geom.bodyid[g];
    if(body<0) {geom_names[g]="ground";continue;}
    if(body>=m->size.nbody)throw std::runtime_error("Invalid native geometry owner");
    const auto name=m->body.nameid[body]>=0 ? m->str.names[m->body.nameid[body]].str() : std::string("world");
    geom_names[g]=name+"_collision_"+std::to_string(geometry_count[body]++);
  }
  // One conservative Coulomb coefficient in this first adapter. No torsional
  // rolling resistance or MuJoCo soft-contact parameters are copied across.
  for(int i=0;i<m->size.ngeom;++i)m->geom.mu[i]=mu;
  crisp::forward(*m,*d);
  std::cout<<std::setprecision(17);
  const auto array=[](const auto& v) {
    std::cout<<'[';for(int k=0;k<v.size();++k){if(k)std::cout<<',';std::cout<<v[k];}std::cout<<']';
  };
  std::cout<<"MODEL {\"bodies\":[";
  for(int j=0;j<m->size.nbody;++j) {
    if(j)std::cout<<',';
    const auto name=m->body.nameid[j]>=0 ? m->str.names[m->body.nameid[j]].str() : std::string("world");
    std::cout<<"{\"name\":\""<<name<<"\",\"position\":";array(d->body.pos[j]);
    std::cout<<",\"quaternion_xyzw\":";array(d->body.quat[j].coeffs());std::cout<<'}';
  }
  std::cout<<"]}"<<std::endl;
  struct ContactSample {
    int geom1,geom2;double gap;Eigen::Vector3d position,normal,force,basis_normal,impulse;
  };
  std::vector<ContactSample> samples;
  for(;;) {
    std::vector<double> q(n),v(n);
    for(int i=0;i<n;++i){q[i]=d->state.q()[qa[i]];v[i]=d->state.v()[va[i]];}
    std::cout<<"STATE {\"time\":"<<d->sim.time<<",\"q\":";array(q);
    std::cout<<",\"v\":";array(v);
    Eigen::Matrix<double,7,1> pose;pose.head<3>()=d->body.pos[ob];pose.tail<4>()=d->body.quat[ob].coeffs();
    std::cout<<",\"object_pose\":";array(pose);
    std::cout<<",\"solver_residual\":"<<d->sol.res<<",\"contacts\":[";
    for(int k=0;k<samples.size();++k) {
      if(k)std::cout<<',';const auto& c=samples[k];
      std::cout<<"{\"pair\":[\""<<geom_names[c.geom1]<<"\",\""
        <<geom_names[c.geom2]<<"\"],\"position\":";array(c.position);
      std::cout<<",\"normal\":";array(c.normal);std::cout<<",\"gap\":"<<c.gap<<",\"wrench\":[0,0,0,"
        <<c.force[0]<<','<<c.force[1]<<','<<c.force[2]<<']';
      if(d->sim.time<.004) {
        std::cout<<",\"native_basis_normal\":";array(c.basis_normal);std::cout<<",\"native_impulse\":";array(c.impulse);
      }
      std::cout<<'}';
    }
    std::cout<<"]}"<<std::endl;
    std::string command;if(!(std::cin>>command) || command=="quit")break;
    if(command!="step")throw std::runtime_error("Unknown bridge command");
    crisp::step1(*m,*d);
    for(int i=0;i<n;++i) {
      double u;if(!(std::cin>>u) || !std::isfinite(u))throw std::runtime_error("Invalid effort");
      d->act.u()[actuator[i]]=u;
    }
    crisp::step2(*m,*d);
    if(!d->state.q().allFinite() || !d->state.v().allFinite())throw std::runtime_error("Nonfinite state");
    samples.clear();
    for(int k=0;k<d->con.lambda.size();++k) {
      const auto& c=d->con.feature[k];
      samples.push_back({c.geom1,c.geom2,c.gap,(c.pos1+c.pos2)*.5,-c.normal,-d->con.rot[k].transpose()*d->con.lambda[k]/dt,d->con.rot[k].row(2).transpose(),d->con.lambda[k]});
    }
    // step2 integrates generalized positions. Refresh body poses so the
    // recorded pose and controller observation share the returned q epoch.
    // Snapshot solver contacts BEFORE refreshing position-dependent data.
    // Never associate a prior solve's impulses with refreshed features.
    crisp::forward(*m,*d);
  }
  return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
