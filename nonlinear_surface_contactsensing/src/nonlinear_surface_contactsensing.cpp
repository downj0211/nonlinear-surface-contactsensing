#include "nonlinear_surface_contactsensing.hpp"

NonlinearSurfaceContactsensing::NonlinearSurfaceContactsensing(ros::NodeHandle nh): nh_(nh){
  cp_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/contactsensing/cp",100);
  ftsensor_sub_ = nh_.subscribe("/ftsensor/measured_value", 100, &NonlinearSurfaceContactsensing::ftsensor_cb, this);
}

void NonlinearSurfaceContactsensing::ftsensor_cb(const std_msgs::Float32MultiArray &msg){
  if(!ft_measure_flag_){
    ft_value_ = msg.data;
    ft_measure_flag_ = true;
  }
}

int NonlinearSurfaceContactsensing::get_ClosestSurface(WrenchAxis r){
  double min_val = INFINITY;
  double dist = 0;
  int idx = 0;

  float x, y, z;
  for(int i = 0; i < surface_shape_.get_Nvertex() ; i++)  {
    surface_shape_.get_Normal(i, x, y, z);

    if ((x * r.f.at[0] + y * r.f.at[1] + z * r.f.at[2]) < 0) {// force should act from outside.
      // calculate the distance between center point and wrench axis.
      surface_shape_.get_CenterPoint(i, x, y, z);

      dist = get_DistanceLinePoint(r, x, y, z);

      if(dist < min_val){min_val = dist; idx = i;}
    }
  }

  return idx;
}

float NonlinearSurfaceContactsensing::get_DistanceLinePoint(WrenchAxis r, float x, float y, float z){
  // find distance line and point.
  float p[3], dist[3], f_norm;

  f_norm = sqrt(r.f.at[0] * r.f.at[0] + r.f.at[1] * r.f.at[1] + r.f.at[2] * r.f.at[2]);
  p[0] = x - r.r0.at[0];  p[1] = y - r.r0.at[1];  p[2] = z - r.r0.at[2];
  dist[0] = r.f.at[1] * p[2] - r.f.at[2] * p[1];
  dist[1] = r.f.at[2] * p[0] - r.f.at[0] * p[2];
  dist[2] = r.f.at[0] * p[1] - r.f.at[1] * p[0];

  return sqrt(dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2]) / f_norm;
}

bool NonlinearSurfaceContactsensing::set_ContactShape(const string& filename, Transformation pos){
  // set the STL file and transformation matrix.
  surface_shape_.STLImport(filename, pos);

  if(surface_shape_.get_Npoint()>0) return true;
  else{
    ROS_ERROR("STLloader Error: can not find STL file");
    return false;
  }
}

void NonlinearSurfaceContactsensing::get_WrenchAxis(std::vector<float> ft, WrenchAxis &out){
  float f_norm = sqrt(ft[0] * ft[0] + ft[1] * ft[1] + ft[2] * ft[2]);

  // force
  insert_wrench(out.f, ft[0] / f_norm, ft[1] / f_norm, ft[2] / f_norm);

  // torque
  if (f_norm == 0)
    insert_wrench(out.r0, 0, 0, 0);
  else {
      insert_wrench(out.r0, (ft[1] * ft[5] - ft[2] * ft[4]) / f_norm,
          (ft[2] * ft[3] - ft[0] * ft[5]) / f_norm, (ft[0] * ft[4] - ft[1] * ft[3]) / f_norm);
  }
}

bool NonlinearSurfaceContactsensing::is_SurfaceContact(int idx, WrenchAxis r, std::vector<int>& vert_idx, std::vector<float> cp)
{
  int f1, f2, f3;
  float* v1, * v2, * v3, * normal;
  v1 = new float[3]; v2 = new float[3]; v3 = new float[3]; normal = new float[3];

  // get all the vectors and normal vector.
  surface_shape_.get_TriangleIndex(idx, f1, f2, f3);
  surface_shape_.get_Triangle(idx, v1, v2, v3, normal);

  // calculate contact point
  float lambda = ((v1[0] * normal[0] + v1[1] * normal[1] + v1[2] * normal[2]) - (r.r0.at[0] * normal[0] + r.r0.at[1] * normal[1] + r.r0.at[2] * normal[2]))
      / (r.f.at[0] * normal[0] + r.f.at[1] * normal[1] + r.f.at[2] * normal[2]);

  float ctp[3], p1[3], p2[3], p3[3], l1[3], l2[3], l3[3];
  for(int i = 0; i < 3; i++){
    cp[0] = r.r0.at[i] + lambda * r.f.at[i];
    ctp[i] = (v1[i] + v2[i] + v3[i]) / 3;

    p1[i] = 0.5 * (v2[i] + v3[i]);
    p2[i] = 0.5 * (v1[i] + v3[i]);
    p3[i] = 0.5 * (v2[i] + v1[i]);

    // calculate the vector of triangle side
    l1[i] = v2[i] - v3[i];
    l2[i] = v1[i] - v3[i];
    l3[i] = v1[i] - v2[i];
  }

  float l1_norm = sqrt(l1[0] * l1[0] + l1[1] * l1[1] + l1[2] * l1[2]);
  float l2_norm = sqrt(l2[0] * l2[0] + l2[1] * l2[1] + l2[2] * l2[2]);
  float l3_norm = sqrt(l3[0] * l3[0] + l3[1] * l3[1] + l3[2] * l3[2]);

  for(int i = 0; i < 3; i++){
    l1[i] = l1[i] / l1_norm;
    l2[i] = l2[i] / l2_norm;
    l3[i] = l3[i] / l3_norm;
  }

  // the direction vector from ctp to p
  float n1[3], n2[3], n3[3];
  for(int i = 0; i < 3; i++){
    n1[i] = (ctp[i] - p1[i])
        - (l1[0] * (ctp[0] - p1[0]) + l1[1] * (ctp[1] - p1[1]) + l1[2] * (ctp[2] - p1[2])) * l1[i];
    n2[i] = (ctp[i] - p2[i])
        - (l2[0] * (ctp[0] - p2[0]) + l2[1] * (ctp[1] - p2[1]) + l2[2] * (ctp[2] - p2[2])) * l2[i];
    n3[0] = (ctp[i] - p3[i])
        - (l3[0] * (ctp[0] - p3[0]) + l3[1] * (ctp[1] - p3[1]) + l3[2] * (ctp[2] - p3[2])) * l3[i];
  }

  //
  int num_positive = 0;
  if ((n1[0] * (cp[0] - p1[0]) + n1[1] * (cp[1] - p1[1]) + n1[2] * (cp[2] - p1[2])) > 0) {
    vert_idx.push_back(f1);
    num_positive++;
  }

  if ((n2[0] * (cp[0] - p2[0]) + n2[1] * (cp[1] - p2[1]) + n2[2] * (cp[2] - p2[2])) > 0) {
    num_positive++;
    vert_idx.push_back(f2);
  }

  if ((n3[0] * (cp[0] - p3[0]) + n3[1] * (cp[1] - p3[1]) + n3[2] * (cp[2] - p3[2])) > 0) {
    num_positive++;
    vert_idx.push_back(f3);
  }

  if (num_positive == 3) return true;
  else return false;
}

int NonlinearSurfaceContactsensing::get_NextTriangle(int idx, std::vector<int>& vert_idx){
  int idx_next = idx;

  int f[3], f_idx[3];
  surface_shape_.get_TriangleIndex(idx, f_idx[0], f_idx[1], f_idx[2]);

  int count_vert = vert_idx.size();
  int size = surface_shape_.get_Nvertex();

  for (int i = 0 ; i < size; i++) {
    surface_shape_.get_TriangleIndex(i, f[0], f[1], f[2]);

    if(((f[0] == vert_idx[0])||(f[1] == vert_idx[0])||(f[2] == vert_idx[0]))&&(idx!=i)) {
        // when side in safe boundary is two
        if(count_vert == 2) {
            if ((f[0] == vert_idx[1]) || (f[1] == vert_idx[1]) || (f[2] == vert_idx[1])) {
                idx_next = i;
                break;
            }
        }
        // when side in safe boundary is one
        else if(count_vert == 1) {
            int count = 0;
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    if (f[j] == f_idx[k])
                        count++;
                }
            }
            if (count == 1) {
                idx_next = i;
                break;
            }
        }
    }
  }

  return idx_next;
}

void NonlinearSurfaceContactsensing::get_ContactPoint(std::vector<float> ft, std::vector<float> cp){
  bool bContact = false;
  int idx, iter = 0;

  if ((abs(ft[0]) + abs(ft[1]) + abs(ft[2]) + abs(ft[3]) + abs(ft[4]) + abs(ft[5])) == 0){
    cp[0] = 0.0;    cp[1] = 0.0;    cp[2] = 0.0;
  }
  else {
    WrenchAxis r;
    get_WrenchAxis(ft, r);
    idx = get_ClosestSurface(r);

    std::vector<int> vert_idx;
    while(!bContact) {
      bContact = is_SurfaceContact(idx, r, vert_idx, cp);

      if(!bContact) idx = get_NextTriangle(idx, vert_idx);

      iter++;

      if(iter > 100) {
        cp[0] = 0.0; cp[1] = 0.0; cp[2] = 0.0;
//        cout << "cannot find contact position in " << iter << "times.\n";
        break;
      }
    }

//    if(bContact)
//      cout << "find contact position in "<<iter << "times.\n";
  }
}


void NonlinearSurfaceContactsensing::run(){
  while(ros::ok()) {
    if(ft_measure_flag_){
      get_ContactPoint(ft_value_, cp_);
      ft_measure_flag_ = false;
    }

    ros::spinOnce();
  }
}
