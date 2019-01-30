/*******************************************************************************
* DO NOT MODIFY - AUTO-GENERATED
* 
* Copyright (c) 2016, McGill University / Independent Robotics Inc.
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* 
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>

#include <aqua_gazebo/aqua_hydrodynamics_plugin.h>
#include "tf/transform_datatypes.h"

union doubleBitHack{
    double d;
    uint64_t i;
};

double sign_func(double x){
	return (x>=0)? ((x==0)?0.0:1.0) :-1.0;
}

uint64_t rotr64 (uint64_t n, unsigned int c){
  const unsigned int mask = (CHAR_BIT*sizeof(n)-1);

  assert ( (c<=mask) &&"rotate by type width or more");
  c &= mask;  // avoid undef behaviour with NDEBUG.  0 overhead for most types / compilers
  return (n>>c) | (n<<( (-c)&mask ));
}

double ComputeVolume(gazebo::physics::LinkPtr _link, ignition::math::Box &bbox){
    unsigned int _index = 0;
    auto _col = _link->GetCollision(_index);
    if(_col){
        auto _shape = _col->GetShape();
        if (_shape->HasType(gazebo::physics::Base::MESH_SHAPE)){
            auto mesh_shape = static_cast<gazebo::physics::MeshShape*>(_shape.get());

            // load mesh using common::MeshManager, since Gazebo does not allow us to acces it :/
            auto mesh_str = mesh_shape->GetMeshURI();
            gazebo::common::MeshManager *meshManager = gazebo::common::MeshManager::Instance();
            auto mesh = meshManager->GetMesh(mesh_str);
            auto _scale = mesh_shape->Size();
            // finally, we can do some computations on the mesh
            //std::cout<<"sub mesh count:"<<mesh->GetSubMeshCount()<<std::endl;
            double volume = 0;
            auto cog = _link->GetInertial()->CoG();

            ignition::math::Vector3<double> _min(1e12,1e12,1e12),_max(-1e12,-1e12,-1e12);
            for (unsigned int i=0; i < mesh->GetSubMeshCount(); i++){
                auto submesh = mesh->GetSubMesh(i);

                // iterate trhough face indices
                ignition::math::Vector3<double> v0,v1,v2;
                for (unsigned int v=0; v < submesh->GetIndexCount();){
                    v0 = submesh->Vertex(submesh->GetIndex(v++))-cog;
                    v1 = submesh->Vertex(submesh->GetIndex(v++))-cog;
                    v2 = submesh->Vertex(submesh->GetIndex(v++))-cog;
                    v0.Set( v0.X()*_scale.X(), v0.Y()*_scale.Y(), v0.Z()*_scale.Z() );
                    v1.Set( v1.X()*_scale.X(), v1.Y()*_scale.Y(), v1.Z()*_scale.Z() );
                    v2.Set( v2.X()*_scale.X(), v2.Y()*_scale.Y(), v2.Z()*_scale.Z() );

                    v0  = v0.Cross(v1);
                    volume += v0.Dot(v2)/6;

                    _min.Min(v0); _min.Min(v1); _min.Min(v2);
                    _max.Max(v0); _max.Max(v1); _max.Max(v2);
                }
            }
            bbox = ignition::math::Box(_min,_max);
            return volume;
        } else if(_shape->HasType(gazebo::physics::Base::BOX_SHAPE)){
            auto box_shape = static_cast<gazebo::physics::BoxShape*>(_shape.get());  
            auto _dims = box_shape->Size();
            return _dims.X()*_dims.Y()*_dims.Z();
        } else if (_shape->HasType(gazebo::physics::Base::SPHERE_SHAPE)){
            auto sphere_shape = static_cast<gazebo::physics::SphereShape*>(_shape.get());  
            double r = sphere_shape->GetRadius();
            return 4.0*M_PI*r*r*r/3.0;
        } else if (_shape->HasType(gazebo::physics::Base::CYLINDER_SHAPE)){
            auto cyl_shape = static_cast<gazebo::physics::CylinderShape*>(_shape.get());
            double r = cyl_shape->GetRadius();
            double h = cyl_shape->GetLength();
            return M_PI*r*r*h;
        }
    }
    return 0;
};

void ComputeHydrodynamicParams(gazebo::physics::LinkPtr _link, HydrodynamicParameters &Hp){
    // allocate memory for K and D
    Hp.K.resize(6*6); Hp.D.resize(6*6);
    Hp.cob.X()=0; Hp.cob.Y()=0; Hp.cob.Z()=0;

    Eigen::Map<Eigen::Matrix<double,6,6> > KT(Hp.K.data(),6,6);
    Eigen::Map< Eigen::Matrix<double,6,6> > DT(Hp.D.data(),6,6);

    unsigned int _index = 0;
    auto _col = _link->GetCollision(_index);
    if(_col){
        auto _shape = _col->GetShape();
        if (_shape->HasType(gazebo::physics::Base::MESH_SHAPE)){
            // we will compute the kirchoff tensor for added mass effects and the drag tensor
            // using the method described in "Underwater Rigid Body Dynamics" by Stefan Weissman and Ulrich Pinkall
            auto mesh_shape = static_cast<gazebo::physics::MeshShape*>(_shape.get());
            auto _scale = mesh_shape->Size();

            // load mesh using common::MeshManager, since Gazebo does not allow us to access it :/
            auto mesh_str = mesh_shape->GetMeshURI();
            gazebo::common::MeshManager *meshManager = gazebo::common::MeshManager::Instance();
            auto mesh = meshManager->GetMesh(mesh_str);

            // finally, we can do some computations on the mesh
            double offset = 1e-9;
            auto cog = _link->GetInertial()->CoG();
            for (unsigned int i=0; i < mesh->GetSubMeshCount(); i++){
                auto submesh = mesh->GetSubMesh(i);

                ignition::math::Vector3<double> v0,v0n;
                std::vector<ignition::math::Vector3<double>> potential_sources;
                std::map<uint64_t,int> vertex_map;
                std::vector<ignition::math::Vector3<double>> vertex_normals;
                std::vector<int> vertex_normal_counts;
                std::vector<ignition::math::Vector3<double>> vertices;
                std::vector<int> vertex_indices;

                std::vector<std::vector<ignition::math::Vector3<double>>> faces;
                std::vector<ignition::math::Vector3<double>> area_v;
                std::vector<ignition::math::Vector3<double>> angular_v;
                std::vector<ignition::math::Vector3<double>> face_normals;
                std::vector<ignition::math::Vector3<double>> face_centers;
                std::vector<double> face_areas;

                // precomputations
                for (unsigned int f=0; f < submesh->GetIndexCount()/3; f++){
                    std::vector<ignition::math::Vector3<double>> v_f;
                    for ( int i=0; i<3; i++){
                        v_f.push_back( submesh->Vertex( submesh->GetIndex(3*f+i) ) - cog );
                        v_f[i].Set( v_f[i].X()*_scale.X(), v_f[i].Y()*_scale.Y(), v_f[i].Z()*_scale.Z() );
                    }
                    
                    // precomputations
                    // compute some quatities we need to estimate the velocity potential
                    // and the kirchoff and drag tensors
                    face_centers.push_back((v_f[0]+v_f[1]+v_f[2])/3);             // face center
                    area_v.push_back(0.5*(v_f[1]-v_f[0]).Cross(v_f[2]-v_f[0]));   // area vector
                    face_areas.push_back(area_v[f].Length());               // area of the triangle
                    face_normals.push_back(area_v[f]/face_areas[f]);              // normal vector
                    angular_v.push_back(face_centers[f].Cross(area_v[f]));        // angular vector
                    faces.push_back(v_f);

                    // vertex normals (average_of_face_normals)  and remove duplicate vertices
                    for (unsigned int v=0; v < 3; v++){
                        // compute key for vertex index (to avoid duplicates)
                        doubleBitHack xx,yy,zz;
                        xx.d = v_f[v].X(); yy.d = v_f[v].Y(); zz.d = v_f[v].Z();
                        //uint64_t v_idx_key = ((xx.i)%(4294967295)*2LL) ^ ((yy.i)%(4294967295)*3LL) ^ ((zz.i)%(4294967295)*7LL); // TODO find another hash function for this
                        uint64_t v_idx_key = rotr64(xx.i,13) ^ rotr64(yy.i,29) ^ rotr64(zz.i,41); // TODO find another hash function for this

                        // if not already visited
                        if( vertex_map.find( v_idx_key ) == vertex_map.end() ){
                            int v_idx = vertex_map.size();
                            // add new index
                            //std::cout<<"A"<<v_idx<<": "<<v_f[v]<<", "<<v_idx_key<<std::endl;
                            vertex_map[v_idx_key] = v_idx;
                            vertex_normals.push_back( face_normals[f] );
                            vertex_normal_counts.push_back( 1 );
                            vertex_indices.push_back( v_idx );
                            vertices.push_back( v_f[v] );
                        } else {
                            int v_idx = vertex_map[ v_idx_key ];
                            //std::cout<<"F"<<v_idx<<": "<<v_f[v]<<", "<<v_idx_key<<std::endl;
                            vertex_normals[ v_idx ] += face_normals[f];
                            vertex_normal_counts[ v_idx ]++;
                            vertex_indices.push_back(v_idx);
                        }
                    }
                }

                std::cout<<"total faces: "<<faces.size()<<std::endl;
                std::cout<<"total vertices: "<<vertices.size()<<std::endl;
                // iterate through vertices to precompute the scalar field point source locations
                for (unsigned int v=0; v < vertices.size(); v++){
                    v0 = vertices[v];
                    v0n = vertex_normals[v]/vertex_normal_counts[v];
                    v0n /= v0n.Length();
                    vertex_normals[v] = v0n;
                    potential_sources.push_back(v0 - offset*v0n);
                    // compute cob
                    Hp.cob += v0;
                    //std::cout<<v0<<", "<<v0n<<std::endl;
                }
                Hp.cob /= potential_sources.size();

                Eigen::Matrix<double,Eigen::Dynamic,6> F(submesh->GetIndexCount()/3, 6);
                Eigen::MatrixXd M(submesh->GetIndexCount()/3, potential_sources.size());
                Eigen::MatrixXd inv_c2s_dist(submesh->GetIndexCount()/3, potential_sources.size());
                std::vector< Eigen::Matrix<double,Eigen::Dynamic,3> > c2s;

                Eigen::Matrix<double,6,Eigen::Dynamic> Q(6,submesh->GetIndexCount()/3);
                // iterate through faces
                //std::cout<<"Building linear system"<<std::endl;
                for (unsigned int f=0; f < submesh->GetIndexCount()/3; f++){
                    auto v_f = faces[f];
                    // normal flux ( the components are for each of the six basis velocities )
                    F(f,0) = angular_v[f].X(); F(f,1) = angular_v[f].Y(); F(f,2) = angular_v[f].Z();
                    F(f,3) = area_v[f].X(); F(f,4) = area_v[f].Y(); F(f,5) = area_v[f].Z();

                    // solid angle using the algorithm of Van Oosteroom (1983), for this face for every point source location
                    // we also compute the distance from the face center to each source location in this loop
                    Eigen::Matrix<double,Eigen::Dynamic,3> c2s_f( potential_sources.size(),3);
                    for (unsigned int s=0; s < potential_sources.size(); s++ ){
                        auto s0 = potential_sources[s];
                        auto r0 = v_f[0]-s0, r1 = v_f[1]-s0, r2 = v_f[2]-s0;
                        auto l0 = r0.Length(), l1 = r1.Length(), l2 = r2.Length();
                        // this is scaled by potential_sources.size() for numerical stability with bigger meshes
                        M(f,s) = 2.0*potential_sources.size()*std::atan2(r0.Dot(r1.Cross(r2)), 
                                            l0*l1*l2 +
                                            r0.Dot(r1)*l2 +
                                            r2.Dot(r0)*l1 +
                                            r1.Dot(r2)*l0);

                        // these are for computing the potential and its gradient at every face
                        auto d_fs = face_centers[f]-s0;
                        c2s_f(s,0) = d_fs.X(); c2s_f(s,1) = d_fs.Y(); c2s_f(s,2) = d_fs.Z();
                        inv_c2s_dist(f,s) = 1.0/d_fs.Length();
                        c2s_f.row(s) *= inv_c2s_dist(f,s)*inv_c2s_dist(f,s)*inv_c2s_dist(f,s);
                    }
                    c2s.push_back(c2s_f);

                    // one point quadrature coeffs (note this is actually exactly the same as F, but this is only
                    // true if the fluid is still.)
                    Q(0,f) = angular_v[f].X(); Q(1,f) = angular_v[f].Y(); Q(2,f) = angular_v[f].Z();
                    Q(3,f) = area_v[f].X(); Q(4,f) = area_v[f].Y(); Q(5,f) = area_v[f].Z();
                }
                // after doing this, we can solve for the strength of the point potential sources ( Neumann problem )
                //std::cout<<"Building linear solver"<<std::endl;
                Eigen::MatrixXd MtM = M.transpose()*M;
                Eigen::MatrixXd MtF = M.transpose()*F;
                Eigen::LDLT<Eigen::MatrixXd> Msolver(MtM);

                auto sigma = Msolver.solve(MtF); // sigma has a component for each one of the six basis velocities
                
                // evaluate the scalar potential at every face center 
                auto phi = potential_sources.size()*inv_c2s_dist*sigma;

                // use one point quadrature ( i.e. area of triangle times potential at face center) to evaluate
                // the linear and angular momenta for each of the six basis velocities and compute the Kirchoff tensor
                KT = Q*phi; // yay! we have added mass for this mesh
                                
                // we can now use the source intensities ( sigma ) to compute the drag tensor
                DT.fill(0);
                Eigen::Matrix<double,3,6> wxzpv = Eigen::Matrix<double,3,6>::Zero();
                wxzpv(0,3) = 1; wxzpv(1,4) = 1; wxzpv(2,5) = 1;
                Eigen::Matrix<double,3,6> d_ = Eigen::Matrix<double,3,6>::Zero();
                Eigen::Matrix<double,3,6> t_ = Eigen::Matrix<double,3,6>::Zero();

                for (unsigned int f=0; f < submesh->GetIndexCount()/3; f++){
                    // w X z + v ( for each of the unit basis velocities)
                    wxzpv(1,0) = -face_centers[f].Z(); wxzpv(2,0) = face_centers[f].Y();
                    wxzpv(0,1) = face_centers[f].Z(); wxzpv(2,1) = -face_centers[f].X();
                    wxzpv(0,2) = -face_centers[f].Y(); wxzpv(1,2) = face_centers[f].X();
					
                    // compute the linear and angular drag components
					// TODO add a component proportional to the square of the velocities (ellipsoid approx?)
                    d_ = -c2s[f].transpose()*sigma - wxzpv;
					//d_ = d_.array().square() * (d_.unaryExpr(std::ptr_fun(sign_func))).array();
                    t_ = wxzpv.leftCols<3>().transpose()*d_;

                    // accumulate for each face
                    DT.row(0) += t_.row(0)*face_areas[f];
                    DT.row(1) += t_.row(1)*face_areas[f];
                    DT.row(2) += t_.row(2)*face_areas[f];
                    DT.row(3) += d_.row(0)*face_areas[f];
                    DT.row(4) += d_.row(1)*face_areas[f];
                    DT.row(5) += d_.row(2)*face_areas[f];
                }
            }
        } else if (_shape->HasType(gazebo::physics::Base::SPHERE_SHAPE)){
            auto sphere_shape = static_cast<gazebo::physics::SphereShape*>(_shape.get());  
            double r = sphere_shape->GetRadius();
            KT.block(3,3,3,3) = Eigen::MatrixXd::Identity(3,3)*4.0*M_PI*r*r*r/3.0;
        } else if (_shape->HasType(gazebo::physics::Base::CYLINDER_SHAPE)){
            // TODO
        } else if(_shape->HasType(gazebo::physics::Base::BOX_SHAPE)){
            // get the corners of the box
        }
    }
};

AquaHydrodynamicsPlugin::AquaHydrodynamicsPlugin(){
    
}

void AquaHydrodynamicsPlugin::InitDisturbances(
        int freq_components, double freq_noise, Eigen::MatrixXd vel_mean,
        Eigen::MatrixXd vel_cov){
    
    // init frequency components from an uniform distribution
    disturbance_spectrum.freqs.resize(freq_components);
    std::generate(disturbance_spectrum.freqs.begin(),
                  disturbance_spectrum.freqs.end(),
                  std::bind(Ud, gen));

    // the disturbance amplitudes will follow something that looks
    disturbance_spectrum.ampls.resize(freq_components);
    int i=0;
    std::generate(disturbance_spectrum.ampls.begin(),
                  disturbance_spectrum.ampls.end(),
                  [&i, m=std::log(0.05), s=1.5, this](){
                      auto x = disturbance_spectrum.freqs[i++];
                      auto p = std::exp(-0.5*((std::log(x)-m)/s));
                      return p*p;
                  });

    disturbance_spectrum.noise = freq_noise;

    disturbance_vel.mean = vel_mean;
    disturbance_vel.cov = vel_cov;
}

void AquaHydrodynamicsPlugin::Load(
        gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf){
    // get parameters from SDF file (gazebo tags in the URDF file)
	if(_sdf->HasElement("planar")){
		planar = _sdf->Get<bool>("planar");
		if( planar )
			ROS_INFO( "planar true ");
		else
			ROS_INFO( "planar false.");
	}
	else{
		planar = false;
		ROS_INFO("aqua hydrodynamics plugin missing <planar>, defaults to false.");
	}

    if (_sdf->HasElement("robotNamespace"))
        robot_namespace = _sdf->Get<std::string>("robotNamespace") + "/";
    else{
        ROS_INFO("aqua hydrodynamics plugin missing <robotNameSpace>, defaults to /aqua");
        robot_namespace = "aqua";
    }

    if (_sdf->HasElement("surfaceLevel")){
        surface_level = _sdf->Get<double>("surfaceLevel");
    } else {
        surface_level = 0.0;
        ROS_INFO("aqua hydrodynamics plugin missing <surfaceLevel>, defaults to %f", surface_level);
    }
    if (_sdf->HasElement("fluidDensity")){
        fluid_density = _sdf->Get<double>("fluidDensity");
        ROS_INFO( "Density loaded from sdf was %f.", fluid_density );
    } else {
        fluid_density = 999.97;
        ROS_INFO("aqua hydrodynamics plugin missing <fluidDensity>, defaults to %f", fluid_density);
    }
    if (_sdf->HasElement("fluidViscosity")){
        fluid_viscosity = _sdf->Get<double>("fluidViscosity");
        ROS_INFO( "Viscosity loaded from sdf was %f.", fluid_viscosity);
    } else {
        fluid_viscosity = 0.1;
        ROS_INFO("aqua hydrodynamics plugin missing <fluidViscosity>, defaults to %f", fluid_viscosity);
    }
    if (_sdf->HasElement("wobble")){
        wobble = _sdf->Get<double>("wobble");
    } else {
        wobble = 1;
        ROS_INFO("aqua hydrodynamics plugin missing <wobble>, defaults to %f", wobble);
    }
    if (_sdf->HasElement("legDragCoeffs")){
        leg_drag_coeffs = _sdf->Get<ignition::math::Vector3<double>>("legDragCoeffs");
    } else {
        leg_drag_coeffs = ignition::math::Vector3<double>(0.0,0.0,1.12);
        ROS_INFO_STREAM("aqua hydrodynamics plugin missing <legDragCoeffs>, defaults to "<<leg_drag_coeffs);
    }
    if (_sdf->HasElement("motorPidGains")){
        pid_gains = _sdf->Get< ignition::math::Vector3<double> >("motorPidGains");
    } else {
        pid_gains = ignition::math::Vector3<double>(0.0,0.0,0.0);
        ROS_INFO_STREAM("aqua flippers plugin missing <motorPidGains>, defaults to "<<pid_gains);
    }

    // init random disturbances
    // make the robot drift a bit
    dist_force = Eigen::Vector3d::Zero();
    prev_vel = Eigen::Vector3d::Zero();
    std::uniform_real_distribution<> d{0.0, 1.0};
    Eigen::VectorXd vel_mean(3);
    vel_mean << Ud(gen), Ud(gen), Ud(gen);
    vel_mean = vel_mean.array()*0.025;
    Eigen::MatrixXd vel_cov(3,3);
    vel_cov << 0.01, 0, 0,
               0, 0.01, 0,
               0, 0, 0.01;

    InitDisturbances(25, 0.0001,  vel_mean, vel_cov);

    // get node handle
    nh_ = new ros::NodeHandle(robot_namespace);

    // setup publisher for params
    hparams_pub = nh_->advertise<aqua_gazebo::HydrodynamicsParamsList>("hydrodynamics_params", 1);
    // setup publisher for thrust params
    tparams_pub = nh_->advertise<aqua_gazebo::ThrustParams>("thrust_params", 1);

    // setup dynamic reconfigure
    server_ = new dynamic_reconfigure::Server<aqua_gazebo::HydrodynamicsConfig>(ros::NodeHandle(robot_namespace+"/hydrodynamics_plugin"));
    dynamic_reconfigure::Server<aqua_gazebo::HydrodynamicsConfig>::CallbackType cb;
    cb = boost::bind(&AquaHydrodynamicsPlugin::DynamicReconfigureCallback, this, _1, _2);
    server_->setCallback(cb);

    // setup service servers for flipper state
    get_leg_command_service = nh_->advertiseService("set_leg_command", &AquaHydrodynamicsPlugin::SetPeriodicLegCommand_cb,this);
    set_leg_command_service = nh_->advertiseService("get_leg_command", &AquaHydrodynamicsPlugin::GetPeriodicLegCommand_cb,this);
    get_target_angles_service = nh_->advertiseService("set_target_joint_angles", &AquaHydrodynamicsPlugin::SetTargetLegAngles_cb,this);
    set_target_angles_service = nh_->advertiseService("get_target_joint_angles", &AquaHydrodynamicsPlugin::GetTargetLegAngles_cb,this);
    
    // Load parameters from SDF file
    ROS_INFO("Loading the Aqua Hydrodynamics plugin");
    model = _parent;
    robot_namespace.clear();
    target_angles.fill(0.0);
    frequency_cmd.fill(0.0);
    amplitude_cmd.fill(0.0);
    leg_offsets_cmd.fill(0.0);
    phase_offsets_cmd.fill(0.0);

    // set initial params
    aqua_gazebo::HydrodynamicsConfig config;
    config.density = fluid_density;
    config.viscosity = fluid_viscosity;
    config.wobble = wobble;
    config.thrustK1 = thrustK1;
    config.thrustK2 = thrustK2;
    DynamicReconfigureCallback(config,0);

    // this event is triggered called on every simulation iteration
    updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&AquaHydrodynamicsPlugin::OnUpdate, this, _1));

    base_link = model->GetLink("aqua_base");
    motor_joints.resize(NUM_LEGS);
    leg_links.resize(NUM_LEGS);
    shoulder_links.resize(NUM_LEGS);
    pid.resize(NUM_LEGS);
    for (size_t i=0;i<NUM_LEGS;++i) {
        leg_links[i] = model->GetLink(LEG_NAMES[i]);
        shoulder_links[i] = model->GetLink(SHOULDER_NAMES[i]);
        motor_joints[i] = model->GetJoint(JOINT_NAMES[i]);
        pid[i].Init(pid_gains.X(), pid_gains.Y(), pid_gains.Z(), 20.0, -20.0, 20.0, -20.0);
    }

    // get leg dimensions
    leg_bbox = leg_links[0]->BoundingBox();

    // compute volume for the rest of the body (approximated by the bounding boxes)
    aqua_volume = 0;
    for(auto &_link : model->GetLinks()){
        HydrodynamicParameters Hp;
        ignition::math::Box _link_bbox;
        auto vol = ComputeVolume(_link, _link_bbox);
        ComputeHydrodynamicParams(_link,Hp);
        Hp.lambda = wobble;
        Hp.fluid_density = fluid_density;
        Hp.fluid_viscosity = fluid_viscosity;
        Hp.volume = vol;
        if(_link->GetName().find("leg") == std::string::npos){ // if this is not a leg
            aqua_volume += vol;
            aqua_bbox.Merge(_link_bbox);
        } else {
            // ignore added mass for the legs
            //Eigen::Map< Eigen::Matrix<double,6,6> > K(Hp.K.data(),6,6);
            //K *= 0;
        }
        hydrodynamic_parameters[_link->GetName()] = Hp;
        
    }

    std::cout<<"leg dimensions: "<<leg_bbox.XLength()<<", "<<leg_bbox.YLength()<<", "<<leg_bbox.ZLength()<<std::endl;
    std::cout<<"aqua dimensions: "<<aqua_bbox.XLength()<<", "<<aqua_bbox.YLength()<<", "<<aqua_bbox.ZLength()<<std::endl;
    std::cout<<"aqua volume: "<<aqua_volume<<" m^3 "<<std::endl;
    std::cout<<"aqua mass: "<<base_link->GetInertial()->Mass()<<" kg "<<std::endl;
    std::cout<<"combined leg volume: "<<6*leg_bbox.XLength()*leg_bbox.YLength()*leg_bbox.ZLength()<<" m^3 "<<std::endl;
    std::cout<<"combined leg mass: "<<6*leg_links[0]->GetInertial()->Mass()<<" kg "<<std::endl;

    world = model->GetWorld();
    current_time = world->SimTime();
    last_update_time =  world->SimTime();

    initialPose = base_link->WorldCoGPose();
    current_pose = base_link->WorldPose();
}


void AquaHydrodynamicsPlugin::OnUpdate(const gazebo::common::UpdateInfo & info){
    // get current pose
    current_pose = base_link->WorldPose();
    // get the current value of gravity
    auto gravity = world->Gravity();
    // get current simulation time
    auto previous_time = current_time;
    current_time = world->SimTime();
    // get aqua's velocities
    auto aqua_lin_vel = current_pose.Rot().RotateVectorReverse(base_link->WorldCoGLinearVel());
    auto aqua_ang_vel = base_link->RelativeAngularVel();

    for(auto &_link : model->GetLinks()){
        if(hydrodynamic_parameters.find(_link->GetName()) != hydrodynamic_parameters.end()){
            // get the hydrodynamic params
            auto Hp =  hydrodynamic_parameters[_link->GetName()];
            // get pose and cog
            auto link_pose = _link->WorldPose();
            auto cog = _link->GetInertial()->CoG();
            // get the relative velocities of the link
            auto w = _link->RelativeAngularVel();
            auto v = _link->RelativeLinearVel();
            //auto v = _link_pose.rot.RotateVectorReverse(_link->GetWorldCoGLinearVel());
            Eigen::Matrix<double,6,1> vel;
            Eigen::Matrix<double,6,1> sq_vel;
            vel << w.X(),w.Y(),w.Z(),v.X(),v.Y(),v.Z();
			sq_vel = vel.array().square() * vel.unaryExpr(std::ptr_fun(sign_func)).array();

            double link_depth = link_pose.Pos().Z();
            // if the robot is above the surface of water set the fluid density to that of air
            double rho = 1.225;
            double nu = 1.983e-5/rho;
            if(link_depth <= surface_level){ 
                // below surface of water
                rho = Hp.fluid_density;
                nu = Hp.fluid_viscosity;
            } 

            // added mass (using the method of http://dl.acm.org/citation.cfm?id=2185600
            if (Hp.K.size() == 36){
                // compute the momemtum due to added mass effects (which depend on the velocities of the link)
                Eigen::Map< Eigen::Matrix<double,6,6> > K(Hp.K.data(),6,6);
				double lambda_ = Hp.lambda;
				K.block<3,3>(0,0) *= lambda_*lambda_;
				K.block<3,3>(3,0) *= lambda_;
				K.block<3,3>(0,3) *= lambda_;
                Eigen::Matrix<double,6,1> momentum = rho*K*vel;
                // compute the added mas force due to the motion of the link
                ignition::math::Vector3<double> l( momentum(0), momentum(1), momentum(2) );
                ignition::math::Vector3<double> p( momentum(3), momentum(4), momentum(5) );
                auto dl = l.Cross(w) + p.Cross(v);
                auto dp = p.Cross(w);

				// TODO  I'm not sure this is entirely correct. I'm adding the Kf.dot([w,v]) term as an external force
				// the correct thing to do is to implement the integrator from http://www.geometry.caltech.edu/pubs/KCD09.pdf
                _link->AddRelativeTorque( dl ) ;
                _link->AddRelativeForce( dp );
            }

            // drag
            if (Hp.D.size() == 36){
                // compute the drag forces
                Eigen::Map< Eigen::Matrix<double,6,6> > D(Hp.D.data(),6,6);
				double drag_scaling_ = Hp.drag_scaling;
				D.block<3,3>(0,0) *= drag_scaling_*drag_scaling_;
				D.block<3,3>(3,0) *= drag_scaling_;
				D.block<3,3>(0,3) *= drag_scaling_;
                Eigen::Matrix<double,6,1> drag = nu*D*vel;
                ignition::math::Vector3<double> t_( drag(0), drag(1), drag(2) );
                ignition::math::Vector3<double> d_( drag(3), drag(4), drag(5) );
                
                // apply them
                _link->AddRelativeTorque( t_ ) ;
                _link->AddRelativeForce( d_ );
            }

            // buoyancy
            _link->AddForce(-rho*Hp.volume*gravity);
        }
    }

    // motor logic
    double dt = current_time.Double() - last_update_time.Double();
    last_update_time = current_time;

    // get the motor commands here!
    for (size_t i=0;i<NUM_LEGS;++i) {
        double joint_angle = std::fmod(motor_joints[i]->Position(1), TWO_M_PI);
        target_angles[i] = std::fmod(target_angles[i],TWO_M_PI);

        //---------------------------------- THRUST MODEL --------------------------------------//
        // here we have the thrust model of [Giguère et al., 2006], [Plamondon & Nahon, 2009], or [Georgiades, 2005]
        auto leg_velocity = leg_links[i]->RelativeLinearVel();

        // just for fun, if the leg is above the surface of water (probably negligible at the given air density)
        double rho = 1.225;
        if(leg_links[i]->WorldCoGPose().Pos().Z() <= surface_level){ 
           // below surface of water
            rho = fluid_density;
        } 
        
        // only apply thrust if it is going to be greater than zero
        if (amplitude_cmd[i] > 0.0 && frequency_cmd[i] > 0.0){
            double thrust = thrustK1*((leg_bbox.YLength() + 2*1.05*leg_bbox.YLength())*leg_bbox.XLength()*leg_bbox.XLength()/3)*rho*std::fabs(amplitude_cmd[i])*frequency_cmd[i] + thrustK2;
            ignition::math::Vector3<double> leg_thrust(thrust,0,0);
            // apply force in the direction of the offset at the hip
            leg_links[i]->AddRelativeForce(leg_thrust);
        }
        
        //---------------------------------- MOTOR CONTROLLER --------------------------------------//
        double error = 0;
        
        pid[i].SetCmd(target_angles[i]);
        error = joint_angle - target_angles[i];

        // limit the error to be between -180 and 180
        error = std::fmod((error + M_PI),TWO_M_PI) - M_PI;
        if (error < -M_PI)
            error += TWO_M_PI;

        pid[i].Update(error, dt);

        motor_joints[i]->SetForce(0, pid[i].GetCmd());
    }

    auto base_link_pose = base_link->WorldPose();
    double base_link_depth = base_link_pose.Pos().Z();
    if(base_link_depth <= surface_level){ 
        // add randomized external disturbances to base link
        Eigen::Vector3d accum_vel = Eigen::Vector3d::Zero();
        auto n=disturbance_spectrum.freqs.size();
        for(unsigned int i=0; i<n; i++){
            // sample velocity
            auto vel = disturbance_vel.sample();
            auto ampl = disturbance_spectrum.ampls[i];
            auto freq = disturbance_spectrum.freqs[i];
            auto t = last_update_time.Double();

            freq += Nd(gen)*disturbance_spectrum.noise;
            vel = vel.array() + vel.minCoeff()*ampl*std::sin(TWO_M_PI*freq*t);
            accum_vel += vel/n;
        }
        double m = base_link->GetInertial()->Mass();
        auto accel = (accum_vel-prev_vel).array()/dt;
        accel.cwiseMin(Eigen::Vector3d::Ones().array()*0.0005);
        accel.cwiseMax(-Eigen::Vector3d::Ones().array()*0.0005);
        dist_force = 0.95*dist_force.array() + m*accel;
        prev_vel += accum_vel-prev_vel;
        base_link->AddForce(
            ignition::math::Vector3<double>(
                0.1*dist_force(0), 0.1*dist_force(1), 0.1*dist_force(2)));
    }

    if( planar ){
        auto wp = base_link->WorldPose();
	    auto ori = wp.Rot();
	    auto eul_vec =  wp.Rot().Euler();
	    eul_vec.Set( 0.0, 0.0, eul_vec[2] );
	    wp.Rot().Euler( eul_vec );

	    wp.Pos().Set( wp.Pos()[0], wp.Pos()[1], -1.0 );

	    base_link->SetWorldPose( wp );
    }
}

void AquaHydrodynamicsPlugin::DynamicReconfigureCallback(aqua_gazebo::HydrodynamicsConfig &config, uint32_t level){
    ROS_INFO("density: [ %f ] viscosity: [ %f ] wobble: [ %f ] K1: [ %f ] K2: [ %f ]", config.density, config.viscosity,config.wobble,config.thrustK1,config.thrustK2);

    aqua_gazebo::HydrodynamicsParamsList params_list;
    params_list.header.stamp = ros::Time::now();

    aqua_gazebo::ThrustParams tparams;
    tparams.header.stamp = ros::Time::now();
 
    // update hydrodynamic params
    fluid_density = config.density;
    fluid_viscosity = config.viscosity;
    wobble = config.wobble;
    drag_scaling = config.drag_scaling;

    for( auto it = hydrodynamic_parameters.begin(); it != hydrodynamic_parameters.end(); it++){
        aqua_gazebo::HydrodynamicsParams params;
        params.header.stamp = params_list.header.stamp;
        params.name = it->first;

        // get the hydrodynamic params
        auto& Hp = it->second;
        Hp.lambda = wobble;
        Hp.drag_scaling = drag_scaling;
        Hp.fluid_density = fluid_density;
        Hp.fluid_viscosity = fluid_viscosity;

        params.wobble = wobble;
        params.drag_scaling = drag_scaling;
        params.density = fluid_density;
        params.viscosity = fluid_viscosity;
        params.mass = Hp.mass;
        params.volume = Hp.volume;
        
        auto com = Hp.com;
        params.com.push_back(com.X());
        params.com.push_back(com.Y());
        params.com.push_back(com.Z());
        auto cob = Hp.cob;
        params.cob.push_back(cob.X());
        params.cob.push_back(cob.Y());
        params.cob.push_back(cob.Z());

        params_list.params.push_back(params);
    }

    // update thrust params
    thrustK1 = config.thrustK1;
    thrustK2 = config.thrustK2;
    tparams.K1 = thrustK1;
    tparams.K2 = thrustK2;
    
    hparams_pub.publish(params_list);
    tparams_pub.publish(tparams);
}

bool AquaHydrodynamicsPlugin::SetPeriodicLegCommand_cb(aquacore::SetPeriodicLegCommand::Request  &req, aquacore::SetPeriodicLegCommand::Response &res){
  std::copy(req.frequencies.begin(), req.frequencies.end(), frequency_cmd.begin());
  std::copy(req.amplitudes.begin(), req.amplitudes.end(), amplitude_cmd.begin());
  std::copy(req.leg_offsets.begin(), req.leg_offsets.end(), leg_offsets_cmd.begin());
  std::copy(req.phase_offsets.begin(), req.phase_offsets.end(), phase_offsets_cmd.begin());
  return true;
}

bool AquaHydrodynamicsPlugin::GetPeriodicLegCommand_cb(aquacore::GetPeriodicLegCommand::Request  &req, aquacore::GetPeriodicLegCommand::Response &res){
  std::copy(frequency_cmd.begin(), frequency_cmd.end(), res.frequencies.begin());
  std::copy(amplitude_cmd.begin(), amplitude_cmd.end(), res.amplitudes.begin());
  std::copy(leg_offsets_cmd.begin(), leg_offsets_cmd.end(), res.leg_offsets.begin());
  std::copy(phase_offsets_cmd.begin(), phase_offsets_cmd.end(), res.phase_offsets.begin());
  return true;
}

bool AquaHydrodynamicsPlugin::SetTargetLegAngles_cb(aquacore::SetTargetLegAngles::Request  &req, aquacore::SetTargetLegAngles::Response &res){
  std::copy(req.target_angles.begin(), req.target_angles.end(), target_angles.begin());
  return true;
}

bool AquaHydrodynamicsPlugin::GetTargetLegAngles_cb(aquacore::GetTargetLegAngles::Request  &req, aquacore::GetTargetLegAngles::Response &res){
  //
  std::copy(target_angles.begin(), target_angles.end(), res.target_angles.begin());
  return true;
}
GZ_REGISTER_MODEL_PLUGIN(AquaHydrodynamicsPlugin);

