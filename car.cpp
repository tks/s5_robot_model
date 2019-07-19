#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#define TEXTURE_PATH "../textures/"

#ifdef  dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#endif

#define COLIDE_MODE 1

#define SPHERE 1
#define BOX 2
#define CAPSULE 3
#define CYLINDER 4

#define B3M1170 1
#define B3M1040 2

struct ObjectData {
  int type;
  dBodyID id;
  dGeomID geom;
  dReal mass;
  dReal radius;
  dReal lx;
  dReal ly;
  dReal lz;
  dReal x0;
  dReal y0;
  dReal z0;
};

struct JointData {
  int type;
  dJointID id;
  dBodyID* parent;
  dBodyID* child;
  dReal x;
  dReal y;
  dReal z;
  dReal ax;
  dReal ay;
  dReal az;
};

struct ServoData {
  int type;
//  dJointID id;
  dReal stall;
  dReal rpm;
  dReal friction;
};

static dWorldID world;
static dSpaceID space;
static dGeomID  ground;
static dJointGroupID contactgroup;
static dBodyID kBody[13];
static dGeomID kGeom[13];
static dJointID kJoint[13];
static dGeomID ground_box;

dReal speed = (10/(0.05*2*3.141592))*3.141592*2;
//dReal speed = (0.15/(0.05*2*3.141592))*3.141592*2;
dReal fMax = 3.90;// box car
//dReal fMax = 16.0;

//dReal step_width,step_length,step_top,step_x0,step_y0;
dReal step_width = 1;
dReal step_length = 1;
dReal step_top = 0.01;
dReal step_x0 = 0;
dReal step_y0 = 2;

int    box_max = 5;
static ObjectData box[13]  ={
//            type,       id,     geom, mas, rad,   lx,   ly,   lz,     x0,   y0,   z0 
/*hontai  */ { BOX, kBody[0], kGeom[0],4.0,0.0 ,0.500,0.500,0.020, 0.00,0.00,0.15},
/*wheel*/{CYLINDER, kBody[1], kGeom[1],0.01,0.05,0.03,0.000,0.000,  0.26,0.25,0.05},
/*wheel*/{CYLINDER, kBody[2], kGeom[2],0.01,0.05,0.03,0.000,0.000, -0.26,0.25,0.05},
/*wheel*/{CYLINDER, kBody[3], kGeom[3],0.01,0.05,0.03,0.000,0.000,  0.26,-0.25,0.05},
/*wheel*/{CYLINDER, kBody[4], kGeom[4],0.01,0.05,0.03,0.000,0.000, -0.26,-0.25,0.05},
};

int servo_max = 4;
static JointData servo[12] ={
  {B3M1170, kJoint[0],   &box[0].id, &box[1].id, 0.26, 0.25,0.05, 1,0,0},
  {B3M1170, kJoint[1],   &box[0].id, &box[2].id,-0.26, 0.25,0.05, 1,0,0},
  {B3M1170, kJoint[2],   &box[0].id, &box[3].id, 0.26,-0.25,0.05, 1,0,0},
  {B3M1170, kJoint[3],   &box[0].id, &box[4].id,-0.26,-0.25,0.05, 1,0,0},
};

static void nearCallback (void *data, dGeomID o1, dGeomID o2) {
  int i,n;
  //if ( !((ground == o1) || (ground == o2)) ) return;
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

  const int N = 28;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      if (COLIDE_MODE == 1){
        contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
          dContactSoftERP | dContactSoftCFM | dContactApprox1;
        contact[i].surface.mu = 0.5;//dInfinity;
        contact[i].surface.slip1 = 0.1;//0.1;
        contact[i].surface.slip2 = 0.1;//0.1;
        contact[i].surface.soft_erp = 1.0;
        contact[i].surface.soft_cfm = 1e-9;
        dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
        dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
      }

      else if (COLIDE_MODE == 2){

        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 0.8;//dInfinity;
        contact[i].surface.mu2 = 0.2;
        contact[i].surface.bounce = 0.1;//0.1;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 1e-8;
        dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
        dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
      }
    }
  }
}

void SetViewPoint() {
  static float xyz[3] = {step_x0+1.5, step_y0-step_length, 0.2};
  //static float hpr[3] = {90.0, 0.0, 0.0};
  static float hpr[3] = {180.0, 0.0, 0.0};
  dsSetViewpoint(xyz, hpr);
}

void DrawObj(ObjectData* obj) {  
  if (obj->type == BOX) {
  static dReal sides[3] = {obj->lx, obj->ly, obj->lz};
  sides[0] = obj->lx;sides[1]=obj->ly;sides[2]=obj->lz;
  dsDrawBox(dBodyGetPosition(obj->id), dBodyGetRotation(obj->id), sides);
  }else if (obj->type == SPHERE) {
  dsDrawSphere(dBodyGetPosition(obj->id), dBodyGetRotation(obj->id), obj->radius);
  }else if (obj->type == CAPSULE) {
    if(obj->lz != 0.0) {
      dsDrawCapsule(dBodyGetPosition(obj->id), dBodyGetRotation(obj->id), obj->lz, obj->radius);
    }else if(obj->ly != 0.0) {
      dsDrawCapsule(dBodyGetPosition(obj->id), dBodyGetRotation(obj->id), obj->ly, obj->radius);
    }else if(obj->lx != 0.0) {
      dsDrawCapsule(dBodyGetPosition(obj->id), dBodyGetRotation(obj->id), obj->lx, obj->radius);
    }
  }else if (obj->type == CYLINDER) {
    if(obj->lz != 0.0) {
      dsDrawCylinder(dGeomGetPosition(obj->geom), dGeomGetRotation(obj->geom), obj->lz, obj->radius);
    }else if(obj->ly != 0.0) {
      dsDrawCylinder(dGeomGetPosition(obj->geom), dGeomGetRotation(obj->geom), obj->ly, obj->radius);
    }else if(obj->lx != 0.0) {
      dsDrawCylinder(dGeomGetPosition(obj->geom), dGeomGetRotation(obj->geom), obj->lx, obj->radius);
    }
  }
}

void SimLoop(int pause) {
  dSpaceCollide (space,0,&nearCallback);
  dWorldStep(world, 0.01);
  dJointGroupEmpty(contactgroup);
  dVector3 ss;
  dGeomBoxGetLengths (ground_box,ss);
  dsDrawBox (dGeomGetPosition(ground_box),dGeomGetRotation(ground_box),ss);
//  dsSetColor(1, 0, 0);
  for(int i=0; i<box_max; i++) { 
    DrawObj(&box[i]);
  }
}

dsFunctions* SetDrawstuffFunction() {
  static dsFunctions dsfn_params;
  dsfn_params.version = DS_VERSION;
  dsfn_params.start   = &SetViewPoint;
  dsfn_params.step    = &SimLoop;
  dsfn_params.command = NULL;
  dsfn_params.stop    = NULL;
  dsfn_params.path_to_textures = TEXTURE_PATH;
  return &dsfn_params;
}
void InitAndCreateWorld() {
  dInitODE();
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  dWorldSetGravity(world, 0.0, 0.0, -9.8);
  ground = dCreatePlane(space,0,0,1,0);
}
//-----------generate-------
void GeneObject(ObjectData* obj) {
  dMass mass_param;
  dMassSetZero(&mass_param);

  if (obj->type == BOX) {
    obj->id = dBodyCreate(world);
    dBodySetPosition(obj->id, obj->x0, obj->y0, obj->z0);
    dMassSetBoxTotal(&mass_param, obj->mass, obj->lx, obj->ly, obj->lz);
    dBodySetMass(obj->id, &mass_param);
    obj->geom = dCreateBox(space,            obj->lx, obj->ly, obj->lz);
    dGeomSetBody(obj->geom, obj->id);
  }else if (obj->type == SPHERE){
    obj->id = dBodyCreate(world);
    dBodySetPosition(obj->id, obj->x0, obj->y0, obj->z0);
    dMassSetSphereTotal(&mass_param, obj->mass, obj->radius);
    dBodySetMass(obj->id, &mass_param);
    obj->geom = dCreateSphere(space,            obj->radius);
    dGeomSetBody(obj->geom, obj->id);

  }else if (obj->type == CAPSULE){
    static dMatrix3 R;
    obj->id = dBodyCreate(world);
    dBodySetPosition(obj->id, obj->x0, obj->y0, obj->z0);

    if (obj->lz != 0.0) {
      dMassSetCapsuleTotal(&mass_param, obj->mass, 3, obj->lz, obj->radius);
      dBodySetMass(obj->id, &mass_param);
      obj->geom = dCreateCapsule(space,               obj->radius, obj->lz);
      
    }else if (obj->ly != 0.0) {
      dMassSetCapsuleTotal(&mass_param, obj->mass, 2, obj->ly, obj->radius);
      dRFromAxisAndAngle(R, 1.0, 0.0, 0.0, M_PI/2);
      dBodySetMass(obj->id, &mass_param);
      dBodySetRotation(obj->id,R);
      obj->geom = dCreateCapsule(space,               obj->radius, obj->ly);

    }else if (obj->lx != 0.0) {
      dMassSetCapsuleTotal(&mass_param, obj->mass, 1, obj->lx, obj->radius);
      dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, M_PI/2);
      dBodySetMass(obj->id, &mass_param);
      dBodySetRotation(obj->id,R);
      obj->geom = dCreateCapsule(space,               obj->radius, obj->lx);
    }

  }else if (obj->type == CYLINDER){
    static dMatrix3 R;
    obj->id = dBodyCreate(world);
    dBodySetPosition(obj->id, obj->x0, obj->y0, obj->z0);

    if (obj->lz != 0.0) {
      dMassSetCylinderTotal(&mass_param, obj->mass, 3, obj->lz, obj->radius);
      dBodySetMass(obj->id, &mass_param);
      obj->geom = dCreateCylinder(space,               obj->radius, obj->lz);
      
    }else if (obj->ly != 0.0) {
      dMassSetCylinderTotal(&mass_param, obj->mass, 2, obj->ly, obj->radius);
      dRFromAxisAndAngle(R, 1.0, 0.0, 0.0, M_PI/2);
      dBodySetMass(obj->id, &mass_param);
      dBodySetRotation(obj->id,R);
      obj->geom = dCreateCylinder(space,               obj->radius, obj->ly);

    }else if (obj->lx != 0.0) {
      dMassSetCylinderTotal(&mass_param, obj->mass, 1, obj->lx, obj->radius);
      dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, M_PI/2);
      dBodySetMass(obj->id, &mass_param);
      dBodySetRotation(obj->id,R);
      obj->geom = dCreateCylinder(space,               obj->radius, obj->lx);
    }
  }
  dGeomSetBody(obj->geom, obj->id);
}

void GeneJoint(JointData* data) {
  data->id = dJointCreateHinge(world, 0);
  dJointAttach(data->id, *data->parent, *data->child);
  dJointSetHingeAnchor(data->id, data->x, data->y, data->z);
  dJointSetHingeAxis(data->id, data->ax, data->ay, data->az);
  dJointSetHingeParam(data->id, dParamFMax, fMax);
}

int main(int argc, char **argv){
  InitAndCreateWorld();
  for (int i=0; i<box_max; i++) {
    GeneObject(&box[i]);
  }
  for(int i=0;i<servo_max;i++){
    GeneJoint(&servo[i]);
  }
  dJointSetHingeParam (servo[0].id,dParamVel,speed);
  dJointSetHingeParam (servo[1].id,dParamVel,speed);
  dJointSetHingeParam (servo[2].id,dParamVel,speed*2);
  dJointSetHingeParam (servo[3].id,dParamVel,speed*2);
  //ground_box = dCreateBox (space,2,1.5,1);
  ground_box = dCreateBox (space,step_width,step_length,step_top);
  //dMatrix3 R;
  //dRFromAxisAndAngle (R,0,1,0,-0.15);
  //dGeomSetPosition (ground_box,0,10,-0.34);
  dGeomSetPosition (ground_box,step_x0,step_y0,step_top/2);
 // dGeomSetRotation (ground_box,R);

  dsSimulationLoop(argc, argv, 400, 300, SetDrawstuffFunction());

  dWorldDestroy(world);
  dCloseODE();
  return 0;
}

