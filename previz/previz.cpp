//////////////////////////////////////////////////////////////////////////////////
// This is a front end for a set of viewer clases for the Carnegie Mellon
// Motion Capture Database: 
//    
//    http://mocap.cs.cmu.edu/
//
// The original viewer code was downloaded from:
//
//   http://graphics.cs.cmu.edu/software/mocapPlayer.zip
//
// where it is credited to James McCann (Adobe), Jernej Barbic (USC),
// and Yili Zhao (USC). There are also comments in it that suggest
// and Alla Safonova (UPenn) and Kiran Bhat (ILM) also had a hand in writing it.
//
//////////////////////////////////////////////////////////////////////////////////
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <float.h>
#include "SETTINGS.h"
#include "skeleton.h"
#include "displaySkeleton.h"
#include "motion.h"

using namespace std;

struct Light {
  VEC3 position;
  VEC3 color;
  float length; //for area light/soft shadow use
  float width;
  Light(VEC3 pos, VEC3 col, float l, float w) : position(pos), color(col), length(l), width(w) {}
};

struct Triangle {
  VEC3 v1, v2, v3;
  VEC2 uv1, uv2, uv3;
  VEC3 color;
  Triangle(VEC3 vert1, VEC3 vert2, VEC3 vert3, VEC3 col) : v1(vert1), v2(vert2), v3(vert3), color(col) {}
};

struct Cylinder {
  VEC3 start;
  VEC3 end;
  VEC3 color;
  float r;
  Cylinder(VEC3 start_plane, VEC3 end_plane, VEC3 col, float radius) : start(start_plane), end(end_plane), color(col), r(radius) {}
};

struct Circle {
  VEC3 center;
  VEC3 p2;
  VEC3 color;
  float r;
  Circle(VEC3 c, VEC3 p2, VEC3 col, float radius) : center(c), p2(p2), color(col), r(radius) {}
};


// Stick-man classes
DisplaySkeleton displayer;    
Skeleton* skeleton;
Motion* motion;

// int windowWidth = 320;
// int windowHeight = 240;
int windowWidth = 1600;
int windowHeight = 1200;

VEC3 eye;
VEC3 lookingAt;
VEC3 up(0,1,0);

// scene geometry
vector<VEC3> sphereCenters;
vector<float> sphereRadii;
vector<VEC3> sphereColors;
vector<Triangle> triangles;
vector<Light> lights;
vector<Cylinder> cylinder;
vector<VEC3> ballPos;
vector<Circle> circles;

//for shadow samples
int numSamples = 8;
//2:1 ratio
int lightL = 4;
int lightW = 2;

int gridSize = 3;

//for texture maps
int floorWidth = 640;
int floorHeight = 480;
float* floorTexture = new float[3 * floorWidth * floorHeight];

int youngBronWidth = 574;
int youngBronHeight = 800;
float* youngBron = new float[3 * youngBronWidth * youngBronHeight];

int chosenBronWidth = 910;
int chosenBronHeight = 1200;
float* chosenBron = new float[3 * chosenBronWidth * chosenBronHeight];

int SD1Width = 238;
int SD1Height = 370;
float* SD1 = new float[3 * SD1Width * SD1Height];

int SD2Width = 837;
int SD2Height = 1200;
float* SD2 = new float[3 * SD2Width * SD2Height];

int SD3Width = 168;
int SD3Height = 224;
float* SD3 = new float[3 * SD3Width * SD3Height];


VEC3 traceRay(const VEC3& rayPos, const VEC3& rayDir, int depth);
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
void readPPM(const string& filename, int& xRes, int& yRes, float*& values)
{
  // try to open the file
  FILE *fp;
  fp = fopen(filename.c_str(), "rb");
  if (fp == NULL)
  {
    cout << " Could not open file \"" << filename.c_str() << "\" for reading." << endl;
    cout << " Make sure you're not trying to read from a weird location or with a " << endl;
    cout << " strange filename. Bailing ... " << endl;
    exit(0);
  }

  // get the dimensions
  unsigned char newline;
  fscanf(fp, "P6\n%d %d\n255%c", &xRes, &yRes, &newline);
  if (newline != '\n') {
    cout << " The header of " << filename.c_str() << " may be improperly formatted." << endl;
    cout << " The program will continue, but you may want to check your input. " << endl;
  }
  int totalCells = xRes * yRes;

  // grab the pixel values
  unsigned char* pixels = new unsigned char[3 * totalCells];
  fread(pixels, 1, totalCells * 3, fp);

  // copy to a nicer data type
  values = new float[3 * totalCells];
  for (int i = 0; i < 3 * totalCells; i++)
    values[i] = pixels[i];

  // clean up
  delete[] pixels;
  fclose(fp);
  cout << " Read in file " << filename.c_str() << endl;
}
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void writePPM(const string& filename, int& xRes, int& yRes, const float* values)
{
  int totalCells = xRes * yRes;
  unsigned char* pixels = new unsigned char[3 * totalCells];
  for (int i = 0; i < 3 * totalCells; i++)
    pixels[i] = values[i];

  FILE *fp;
  fp = fopen(filename.c_str(), "wb");
  if (fp == NULL)
  {
    cout << " Could not open file \"" << filename.c_str() << "\" for writing." << endl;
    cout << " Make sure you're not trying to write from a weird location or with a " << endl;
    cout << " strange filename. Bailing ... " << endl;
    exit(0);
  }

  fprintf(fp, "P6\n%d %d\n255\n", xRes, yRes);
  fwrite(pixels, 1, totalCells * 3, fp);
  fclose(fp);
  delete[] pixels;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
float normalize(VEC3 v){
  return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

VEC3 cross(VEC3 v1, VEC3 v2){
  return VEC3(
    v1[1] * v2[2] - v1[2] * v2[1],
    v1[2] * v2[0] - v1[0] * v2[2],
    v1[0] * v2[1] - v1[1] * v2[0] 
  );  
}

float dot(VEC3 v1, VEC3 v2){
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]; 
}

VEC3 compDot(VEC3 v1, VEC3 v2){
  return VEC3(v1[0] * v2[0], v1[1] * v2[1], v1[2] * v2[2]);
}

VEC3 truncate(const VEC4& v)
{
  return VEC3(v[0], v[1], v[2]);
}

VEC4 extend(const VEC3& v)
{
  return VEC4(v[0], v[1], v[2], 1.0);
}
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
bool raySphereIntersect(const VEC3& center, 
                        const float radius, 
                        const VEC3& rayPos, 
                        const VEC3& rayDir,
                        float& t)
{
  const VEC3 op = center - rayPos;
  const float eps = 1e-8;
  const float b = op.dot(rayDir);
  float det = b * b - op.dot(op) + radius * radius;

  // determinant check
  if (det < 0) 
    return false; 
  
  det = sqrt(det);
  t = b - det;
  if (t <= eps)
  {
    t = b + det;
    if (t <= eps)
      t = -1;
  }

  if (t < 0) return false;
  return true;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
bool rayTriangleIntersect(const Triangle& triangle, const VEC3& rayPos, const VEC3& rayDir, float& t) {
  //Compute plane normal
  VEC3 v1v2 = triangle.v2 - triangle.v1;
  VEC3 v1v3 = triangle.v3 - triangle.v1;
  
  VEC3 normal = cross(v1v2, v1v3) / normalize(cross(v1v2, v1v3));

  //Check if parallel
  if(fabs(dot(rayDir, normal)) < 1e-4){
    return false;
  }

  float d = -dot(normal, triangle.v1);

  t = -(dot(normal, rayPos) + d) / dot(rayDir, normal);

  //Check if triangle is behind
  if (t < 0){
    return false;
  }

  VEC3 point = rayPos + t * rayDir;

  VEC3 perpN;
  VEC3 v1point = point - triangle.v1;
  perpN = cross(v1v2, v1point);
  if(dot(normal, perpN) < 0){
    return false;
  }

  VEC3 v3v2 = triangle.v3 - triangle.v2;
  VEC3 v2point = point - triangle.v2;
  perpN = cross(v3v2, v2point);
  if(dot(normal, perpN) < 0){
    return false;
  }

  VEC3 v3v1 = triangle.v1 - triangle.v3;
  VEC3 v3point = point - triangle.v3;
  perpN = cross(v3v1, v3point);
  if(dot(normal, perpN) < 0){
    return false;
  }

  return true;
}
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
bool rayCircleIntersect(const Circle& circle, const VEC3& rayPos, const VEC3& rayDir, float& t){

  VEC3 circleAxis = circle.center - circle.p2;
  float height = circleAxis.norm();
  VEC3 normal = circleAxis / height;


	//check if ray is parallel to plane
	if(fabs(dot(rayDir, normal)) < 1e-4){
		return false;
	}

	//(p-c) dot n
	t = dot((circle.center - rayPos), normal) / dot(rayDir, normal);
	if(t < 0){
		return false; //behind rayPos
	}

	VEC3 intersectPoint = rayPos + t * rayDir;

	//check if in circle and if to fill circle or outline
  if(circle.color == VEC3(1, 1, 1)){
    float border = .08;
    if (pow((intersectPoint - circle.center).norm(), 2) <= pow(circle.r, 2) && pow((intersectPoint - circle.center).norm(), 2) >= pow(circle.r - border, 2)){
      return true;
    }
  }
  else{
    if (pow((intersectPoint - circle.center).norm(), 2) <= pow(circle.r, 2)){
      return true;
    }
  }

  return false;
	
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
bool rayCylinderIntersect(const Cylinder& cylinder, const VEC3& rayPos, const VEC3& rayDir, float& t) {
  //calculate the cylinder axis and direction
  VEC3 cylinderAxis = cylinder.end - cylinder.start;
  float height = normalize(cylinderAxis);
  VEC3 direction = cylinderAxis / height; //normalize axis to get direction

  //project the ray direction and the vector from ray origin to cylinder start onto the perpendicular plane
  VEC3 delta_p = rayPos - cylinder.start;
  VEC3 ray_dir_proj = rayDir - direction * dot(rayDir, direction);
  VEC3 delta_p_proj = delta_p - direction * dot(delta_p, direction);

  //solve equation
  float a = dot(ray_dir_proj, ray_dir_proj);
  float b = 2 * dot(ray_dir_proj, delta_p_proj);
  float c = dot(delta_p_proj, delta_p_proj) - cylinder.r * cylinder.r;

  //calculate discriminant
  float discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    return false;
  }

  float sqrt_disc = sqrt(discriminant);
  float t0 = (-b - sqrt_disc) / (2 * a);
  float t1 = (-b + sqrt_disc) / (2 * a);

  //check if intersection points are in cylinder start and end bounds
  float y0 = dot(delta_p, direction) + t0 * dot(rayDir, direction);
  float y1 = dot(delta_p, direction) + t1 * dot(rayDir, direction);

  bool t0_in_bounds = (t0 > 0) && (y0 >= 0 && y0 <= height);
  bool t1_in_bounds = (t1 > 0) && (y1 >= 0 && y1 <= height);

  if (t0_in_bounds && t1_in_bounds) {
    t = min(t0, t1);
    return true;
  } else if (t0_in_bounds) {
    t = t0;
    return true;
  } else if (t1_in_bounds) {
    t = t1;
    return true;
  }

  return false;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
bool isInShadow(VEC3 intersectionPoint, VEC3 shadowRay, Light& light){
  float lightDist = sqrt(dot(light.position - intersectionPoint, light.position - intersectionPoint)); //t has to intersect after or it got blocked

  for(int i = 0; i < sphereCenters.size(); i++){ //we check to see if it got block by any of the spheres
    float t = FLT_MAX;
    if (raySphereIntersect(sphereCenters[i], sphereRadii[i], intersectionPoint + shadowRay * 0.001f, shadowRay, t))
    { 
      if(t < lightDist){
        return true;
      }
    }
  }

  for(int i = 0; i < triangles.size(); i++){ //we check to see if it got block by any of the triangles
    Triangle& triangle = triangles[i];
    float t;
    if(rayTriangleIntersect(triangle, intersectionPoint + shadowRay * 0.001f, shadowRay, t)){ //use shadowRay * 0.0001f to prevent acne
      if(t < lightDist){
        return true;
      }
    }
  }

  for(int i = 0; i < cylinder.size(); i++){ //we check to see if it got block by any of the cylinders
    Cylinder& cyl = cylinder[i];
    float t;
    if(rayCylinderIntersect(cyl, intersectionPoint + shadowRay * 0.001f, shadowRay, t)){ //use shadowRay * 0.0001f to prevent acne
      if(t < lightDist){
        return true;
      }
    }
  }

  for(int i = 0; i < circles.size(); i++){ //we check to see if it got block by any of the cylinders
    Circle& circle = circles[i];
    float t;
    if(rayCircleIntersect(circle, intersectionPoint + shadowRay * 0.001f, shadowRay, t)){ //use shadowRay * 0.0001f to prevent acne
      if(t < lightDist){
        return true;
      }
    }
  }
  return false;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
VEC3 monteCarloLight(Light& light, int sampleNum) {
  
  float cellL = light.length / float(lightL);
  float cellW = light.width / float(lightW);

  float x = sampleNum % lightL * cellL;
  float y = sampleNum / lightW * cellW;

  float jitterX = float(rand()) / float(RAND_MAX) * cellL;
  float jitterY = float(rand()) / float(RAND_MAX) * cellW;


  return VEC3(light.position[0] + x + jitterX, light.position[1], light.position[2] + y + jitterY);
}


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void shadows(const VEC3& rayDir, VEC3& intersectPoint, VEC3& normal, VEC3& pixelColor, VEC3& color, int phong = 100){ //modified for soft shadow
  for(int i = 0; i < lights.size(); i++){
    int unblocked = 0;
    Light& light = lights[i];
    for(int j = 0; j < numSamples; j++){
      VEC3 lightSample = monteCarloLight(light, j);
      VEC3 lightDir = (lightSample - intersectPoint) / normalize(lightSample - intersectPoint);
      if(isInShadow(intersectPoint, lightDir, light)){
        continue;
      }
      else{
        unblocked++;
      }
    }
    VEC3 lightDir = (light.position - intersectPoint) / normalize(light.position - intersectPoint);
    VEC3 reflectDir = 2 * dot(normal, lightDir) * normal - lightDir;
    VEC3 temp = light.color * max(0.0f, dot(normal, lightDir)) + light.color * pow(max(0.0f, dot(reflectDir, -rayDir)), phong);
    pixelColor += (float(unblocked) / float(numSamples)) * compDot(color, temp);

  }
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
VEC3 generateRefract(VEC3 in, VEC3 normal, float alpha1, float alpha2, bool& tir) {
  //T = A + B
  //A = Msin(phi)
  //V= -Ncos(phi)
  //C = cos(theta) * N
  //M = (in + C) / sin(theta)

  float a = alpha1/alpha2;
  float cos01 = -dot(normal, in);
  float sin01 = sqrt(1 - (cos01 * cos01));
  float sin02 = a * sin01;
  float cos02 = sqrt(1 - (sin02 * sin02));
  VEC3 C = cos01 * normal;
  VEC3 M = (in + C) / (sin01);
  VEC3 A = M * sin02;
  VEC3 B = -normal * cos02;

  if((sin02 * sin02) > 1){ //total internal reflection 
    tir = true;
  }

  return A + B;
}




//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void kFresnel(float& kReflect, float& kRefract, float alpha1, float alpha2, VEC3 in, VEC3 normal){
  float a = alpha1/alpha2;
  float cos01 = -dot(normal, in);
  float sin01 = sqrt(1 - (cos01 * cos01));
  float sin02 = a * sin01;
  float cos02 = sqrt(1 - (sin02 * sin02));

  float pPara = (alpha2 * cos01 - alpha1 * cos02) / (alpha2 * cos01 + alpha1 * cos02);
  float pPerp = (alpha1 * cos01 - alpha2 * cos02) / (alpha1 * cos01 + alpha2 * cos02);

  kReflect = .5 * (pPara * pPara + pPerp * pPerp);
  kRefract = 1 - kReflect;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
VEC3 textureMap(Triangle& triangle, VEC3 intersectPoint, float* textureVals, int width, int height){
  //Barycentric Coords
  VEC3 v0 = triangle.v2 - triangle.v1;
  VEC3 v1 = triangle.v3 - triangle.v1;

  float areaAll = 0.5f * normalize(cross(v0, v1));
  float areaA = 0.5f * normalize(cross((triangle.v3 - intersectPoint), (triangle.v2 - intersectPoint)));
  float areaB = 0.5f * normalize(cross((triangle.v3 - intersectPoint), (triangle.v1 - intersectPoint)));
  float areaC = 0.5f * normalize(cross((triangle.v1 - intersectPoint), (triangle.v2 - intersectPoint)));

  float alpha = areaA / areaAll;
  float beta = areaB / areaAll;
  float gamma = areaC / areaAll;

  float uCoord = alpha * triangle.uv1[0] + beta * triangle.uv2[0] + gamma * triangle.uv3[0];
  float vCoord = alpha * triangle.uv1[1] + beta * triangle.uv2[1] + gamma * triangle.uv3[1];
  
  int texCoordX = int(uCoord * (width - 1));
  int texCoordY = int(vCoord * (height - 1));
  int texIndex = 3 * (texCoordY * width + texCoordX);
  
  VEC3 color = VEC3(
      textureVals[texIndex] / 255.0f, 
      textureVals[texIndex + 1] / 255.0f, 
      textureVals[texIndex + 2] / 255.0f
  );
  return color;
}



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void rayColor(const VEC3& rayPos, const VEC3& rayDir, VEC3& pixelColor, int depth) 
{
  pixelColor = VEC3(1,1,1);

  // look for intersections
  int hitIDSphere = -1;
  float tMinFoundSph = FLT_MAX;
  for (int y = 0; y < sphereCenters.size(); y++)
  {
    float tMin = FLT_MAX;
    if (raySphereIntersect(sphereCenters[y], sphereRadii[y], rayPos, rayDir, tMin))
    { 
      // is the closest so far?
      if (tMin < tMinFoundSph)
      {
        tMinFoundSph = tMin;
        hitIDSphere = y;
      }
    }
  }

  int hitIDTri = -1;
  float tMinFoundTri = FLT_MAX;
  for (int y = 0; y < triangles.size(); y++)
  {
    float tMin = FLT_MAX;
    if (rayTriangleIntersect(triangles[y], rayPos, rayDir, tMin))
    { 
      // is the closest so far?
      if (tMin < tMinFoundTri)
      {
        tMinFoundTri = tMin;
        hitIDTri = y;
      }
    }
  }

  int hitIDCyl = -1;
  float tMinFoundCyl = FLT_MAX;
  for (int y = 0; y < cylinder.size(); y++)
  {
    float tMin = FLT_MAX;
    if (rayCylinderIntersect(cylinder[y], rayPos, rayDir, tMin))
    { 
      // is the closest so far?
      if (tMin < tMinFoundCyl)
      {
        tMinFoundCyl = tMin;
        hitIDCyl = y;
      }
    }
  }

  int hitIDCirc = -1;
  float tMinFoundCirc = FLT_MAX;
  for (int y = 0; y < circles.size(); y++)
  {
    float tMin = FLT_MAX;
    if (rayCircleIntersect(circles[y], rayPos, rayDir, tMin))
    { 
      // is the closest so far?
      if (tMin < tMinFoundCirc)
      {
        tMinFoundCirc = tMin;
        hitIDCirc = y;
      }
    }
  }




  // No intersection, return white
  if (hitIDSphere == -1 && hitIDTri == -1 && hitIDCyl == -1 && hitIDCirc == -1){
    return;
  }
  float tMin = min(min(tMinFoundCyl, min(tMinFoundSph, tMinFoundTri)), tMinFoundCirc);
  VEC3 intersectPoint = rayPos + rayDir * tMin;
  VEC3 normal;

  if(tMin == tMinFoundSph){
    pixelColor = VEC3(0, 0, 0);
    normal = (intersectPoint - sphereCenters[hitIDSphere]) / normalize(intersectPoint - sphereCenters[hitIDSphere]);
    if(sphereColors[hitIDSphere] == VEC3(0.0, 1.0, 0.0)){
      
      VEC3 reflectDir = rayDir - 2 * dot(normal, rayDir) * normal;
      VEC3 reflectionColor = traceRay(intersectPoint + reflectDir * 0.001f, reflectDir, depth);
      pixelColor = pixelColor + reflectionColor;
    }
    else{
      shadows(rayDir, intersectPoint, normal, pixelColor, sphereColors[hitIDSphere]);
    }
  }

  else if(tMin == tMinFoundTri){
    Triangle triangle = triangles[hitIDTri];
    VEC3 v1v2 = triangle.v2 - triangle.v1;
    VEC3 v1v3 = triangle.v3 - triangle.v1;
    normal = cross(v1v2, v1v3) / normalize(cross(v1v2, v1v3));

    if(triangle.color == VEC3(.25, .25, 1)){
      VEC3 color = textureMap(triangle, intersectPoint, floorTexture, floorWidth, floorHeight);
      pixelColor = VEC3(0,0,0); 
      shadows(rayDir, intersectPoint, normal, pixelColor, color);
    }
    else if(hitIDTri == 18 || hitIDTri == 19){
      VEC3 color = textureMap(triangle, intersectPoint, youngBron, youngBronWidth, youngBronHeight);
      pixelColor = VEC3(0,0,0); 
      shadows(rayDir, intersectPoint, normal, pixelColor, color);
    }
    else if(hitIDTri == 20 || hitIDTri == 21){
      VEC3 color = textureMap(triangle, intersectPoint, chosenBron, chosenBronWidth, chosenBronHeight);
      pixelColor = VEC3(0,0,0); 
      shadows(rayDir, intersectPoint, normal, pixelColor, color);
    }
    else if(hitIDTri == 22 || hitIDTri == 23){
      VEC3 color = textureMap(triangle, intersectPoint, SD1, SD1Width, SD1Height);
      pixelColor = VEC3(0,0,0); 
      shadows(rayDir, intersectPoint, normal, pixelColor, color);
    }
    else if(hitIDTri == 24 || hitIDTri == 25){
      VEC3 color = textureMap(triangle, intersectPoint, SD2, SD2Width, SD2Height);
      pixelColor = VEC3(0,0,0); 
      shadows(rayDir, intersectPoint, normal, pixelColor, color);
    }
    else if(hitIDTri == 26 || hitIDTri == 27){
      VEC3 color = textureMap(triangle, intersectPoint, SD3, SD3Width, SD3Height);
      pixelColor = VEC3(0,0,0); 
      shadows(rayDir, intersectPoint, normal, pixelColor, color);
    }
    else{
      pixelColor = VEC3(0, 0, 0); 
      shadows(rayDir, intersectPoint, normal, pixelColor, triangle.color);
    }
  }
  else if(tMin == tMinFoundCyl){
    VEC3 cylinderAxis = cylinder[hitIDCyl].end - cylinder[hitIDCyl].start;
    VEC3 projectionOntoAxis = cylinder[hitIDCyl].start + (dot(intersectPoint - cylinder[hitIDCyl].start, cylinderAxis) / dot(cylinderAxis, cylinderAxis)) * cylinderAxis; 
    normal = (intersectPoint - projectionOntoAxis) / normalize(intersectPoint - projectionOntoAxis);

    pixelColor = VEC3(0, 0, 0);
    shadows(rayDir, intersectPoint, normal, pixelColor, cylinder[hitIDCyl].color);
  }
  else if(tMin == tMinFoundCirc){
    pixelColor = VEC3(0,0,0);
    VEC3 circleAxis = circles[hitIDCirc].center - circles[hitIDCirc].p2;
    float height = circleAxis.norm();
    VEC3 normal = circleAxis / height;
    shadows(rayDir, intersectPoint, normal, pixelColor, circles[hitIDCirc].color);
  }
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
VEC3 traceRay(const VEC3& rayPos, const VEC3& rayDir, int depth){
  if (depth > 9){
    return VEC3(0, 0, 0); 
  }

  VEC3 color = VEC3(0, 0, 0);
  rayColor(rayPos, rayDir, color, depth + 1);
  return color;

}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

float clamp(float value)
{
  if (value < 0.0)      return 0.0;
  else if (value > 1.0) return 1.0;
  return value;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void renderImage(int& xRes, int& yRes, const string& filename) 
{
  // allocate the final image
  const int totalCells = xRes * yRes;
  float* ppmOut = new float[3 * totalCells];

  // compute image plane
  const float halfY = (lookingAt - eye).norm() * tan(45.0f / 360.0f * M_PI);
  const float halfX = halfY * 4.0f / 3.0f;

  const VEC3 cameraZ = (lookingAt - eye).normalized();
  const VEC3 cameraX = up.cross(cameraZ).normalized();
  const VEC3 cameraY = cameraZ.cross(cameraX).normalized();

  for (int y = 0; y < yRes; y++) 
    for (int x = 0; x < xRes; x++) 
    {
      VEC3 color = VEC3(0, 0, 0);

      for (int i = 0; i < gridSize; i++) { //stratified sample on each pixel
        for (int j = 0; j < gridSize; j++) {       
          float cellSize = 1.0f / (float)gridSize;
    
          float jitterX = float(rand()) / float(RAND_MAX) * cellSize;
          float jitterY = float(rand()) / float(RAND_MAX) * cellSize;

          float sampleX = x + (j * cellSize) + jitterX;
          float sampleY = y + (i * cellSize) + jitterY;

          float ratioX = 1.0f - sampleX / float(xRes) * 2.0f;
          float ratioY = 1.0f - sampleY / float(yRes) * 2.0f;

          VEC3 rayHitImage = lookingAt +
                             ratioX * halfX * cameraX +
                             ratioY * halfY * cameraY;
          VEC3 rayDir = (rayHitImage - eye).normalized();

          VEC3 sampleColor = VEC3(0, 0, 0);
          rayColor(eye, rayDir, sampleColor, 0);

          color += sampleColor;
        }
      }
      color /= (gridSize * gridSize);

      // set, in final image
      ppmOut[3 * (y * xRes + x)] = clamp(color[0]) * 255.0f;
      ppmOut[3 * (y * xRes + x) + 1] = clamp(color[1]) * 255.0f;
      ppmOut[3 * (y * xRes + x) + 2] = clamp(color[2]) * 255.0f;
    }
  writePPM(filename, xRes, yRes, ppmOut);

  delete[] ppmOut;
}

//////////////////////////////////////////////////////////////////////////////////
// Load up a new motion captured frame
//////////////////////////////////////////////////////////////////////////////////
void setSkeletonsToSpecifiedFrame(int frameIndex)
{
  if (frameIndex < 0)
  {
    printf("Error in SetSkeletonsToSpecifiedFrame: frameIndex %d is illegal.\n", frameIndex);
    exit(0);
  }
  if (displayer.GetSkeletonMotion(0) != NULL)
  {
    int postureID;
    if (frameIndex >= displayer.GetSkeletonMotion(0)->GetNumFrames())
    {
      cout << " We hit the last frame! You might want to pick a different sequence. " << endl;
      postureID = displayer.GetSkeletonMotion(0)->GetNumFrames() - 1;
    }
    else 
      postureID = frameIndex;
    displayer.GetSkeleton(0)->setPosture(* (displayer.GetSkeletonMotion(0)->GetPosture(postureID)));
  }
}

//////////////////////////////////////////////////////////////////////////////////
// Build a list of spheres in the scene
//////////////////////////////////////////////////////////////////////////////////
void buildScene()
{
  sphereCenters.clear();
  sphereRadii.clear();
  sphereColors.clear();
  displayer.ComputeBonePositions(DisplaySkeleton::BONES_AND_LOCAL_FRAMES);
  cylinder.clear();
  circles.clear();

  // retrieve all the bones of the skeleton
  vector<MATRIX4>& rotations = displayer.rotations();
  vector<MATRIX4>& scalings  = displayer.scalings();
  vector<VEC4>& translations = displayer.translations();
  vector<float>& lengths     = displayer.lengths();

  // build a sphere list, but skip the first bone, 
  // it's just the origin
  int totalBones = rotations.size();
  for (int x = 1; x < totalBones; x++)
  {
    MATRIX4& rotation = rotations[x];
    MATRIX4& scaling = scalings[x];
    VEC4& translation = translations[x];

    // get the endpoints of the cylinder
    VEC4 leftVertex(0,0,0,1);
    VEC4 rightVertex(0,0,lengths[x],1);

    leftVertex = rotation * scaling * leftVertex + translation;
    rightVertex = rotation * scaling * rightVertex + translation;
    if(x == 16) {
      //Only need one sphere for the head. Position is middle of endpoints
      VEC3 center = (leftVertex.head<3>() + rightVertex.head<3>()) / 2;

      sphereCenters.push_back(center);
      sphereRadii.push_back(0.111);
      sphereColors.push_back(VEC3(1,0,0));       
    }
    else{
      float cylinderRadius = 0.05;
      VEC3 cylinderColor(.875, .090, .118);
      cylinder.push_back(Cylinder(truncate(leftVertex), truncate(rightVertex), cylinderColor, cylinderRadius));
      circles.push_back(Circle(truncate(rightVertex), truncate(leftVertex), cylinderColor, cylinderRadius));
      circles.push_back(Circle(truncate(leftVertex), truncate(rightVertex), cylinderColor, cylinderRadius));
    }
    // used to calculate ballPos
    // if(x == 21){ //left hand
    //   cout << (leftVertex.head<3>() + rightVertex.head<3>()) / 2 << endl;
    // }
    // if(x == 28){ //right hand
    //   cout << (leftVertex.head<3>() + rightVertex.head<3>()) / 2 << endl;
    // }
  }
}
//////////////////////////////////////////////////////////////////////////////////
// Build floor
//////////////////////////////////////////////////////////////////////////////////
void buildStaticScenes(){
  // floor  
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      triangles.push_back(Triangle(VEC3(1.95 - (3 * j), 0, 2 - (4 * i)), VEC3(1.95 - (3 * j), 0, 6 - (4 * i)), VEC3(4.95 - (3 * j), 0, 6 - (4 * i)), VEC3(.25, .25, 1))); 
      triangles.push_back(Triangle(VEC3(4.95 - (3 * j), 0, 6 - (4 * i)), VEC3(4.95 - (3 * j), 0, 2 - (4 * i)), VEC3(1.95 - (3 * j), 0, 2 - (4 * i)), VEC3(.25, .25, 1)));
      int idx1 = 6 * i + 2 * j;
      int idx2 = 6 * i + 2 * j + 1;
      triangles[idx1].uv1 = VEC2(0, 0);
      triangles[idx1].uv2 = VEC2(1, 0);
      triangles[idx1].uv3 = VEC2(1, 1); 
      triangles[idx2].uv1 = VEC2(1, 1);
      triangles[idx2].uv2 = VEC2(0, 1);
      triangles[idx2].uv3 = VEC2(0, 0); 

    }
  }
  
  //posters
  //young_bron 
  triangles.push_back(Triangle(VEC3(4.94, 3.55, -4.97), VEC3(4.94, 3.55, -3.08), VEC3(4.94, 6.19, -3.08), VEC3(1, .25, 1)));
  triangles.push_back(Triangle(VEC3(4.94, 6.19, -3.08), VEC3(4.94, 6.19, -4.97), VEC3(4.94, 3.55, -4.97), VEC3(1, .25, 1)));
  triangles[18].uv1 = VEC2(0, 1);
  triangles[18].uv2 = VEC2(1, 1);
  triangles[18].uv3 = VEC2(1, 0); 
  triangles[19].uv1 = VEC2(1, 0);
  triangles[19].uv2 = VEC2(0, 0);
  triangles[19].uv3 = VEC2(0, 1); 

  //chosen_bron
  triangles.push_back(Triangle(VEC3(4.94, 1.31, 1.46), VEC3(4.94, 1.31, 3.6), VEC3(4.94, 4.12, 3.6), VEC3(1, .25, 1)));
  triangles.push_back(Triangle(VEC3(4.94, 4.12, 3.6), VEC3(4.94, 4.12, 1.46), VEC3(4.94, 1.31, 1.46), VEC3(1, .25, 1)));
  triangles[20].uv1 = VEC2(0, 1);
  triangles[20].uv2 = VEC2(1, 1);
  triangles[20].uv3 = VEC2(1, 0); 
  triangles[21].uv1 = VEC2(1, 0);
  triangles[21].uv2 = VEC2(0, 0);
  triangles[21].uv3 = VEC2(0, 1); 

  //sd1
  triangles.push_back(Triangle(VEC3(4.941, 3.75, 3.31), VEC3(4.941, 3.75, 4.96), VEC3(4.941, 6.3, 4.96), VEC3(1, .25, 1)));
  triangles.push_back(Triangle(VEC3(4.941, 6.3, 4.96), VEC3(4.941, 6.3, 3.31), VEC3(4.941, 3.75, 3.31), VEC3(1, .25, 1)));
  triangles[22].uv1 = VEC2(0, 1);
  triangles[22].uv2 = VEC2(1, 1);
  triangles[22].uv3 = VEC2(1, 0);
  triangles[23].uv1 = VEC2(1, 0);
  triangles[23].uv2 = VEC2(0, 0);
  triangles[23].uv3 = VEC2(0, 1); 

  //sd2
  triangles.push_back(Triangle(VEC3(4.94, 2.85, -1.16), VEC3(4.94, 2.85, 0.92), VEC3(4.94, 5.81, 0.92), VEC3(1, .25, 1)));
  triangles.push_back(Triangle(VEC3(4.94, 5.81, 0.92), VEC3(4.94, 5.81, -1.16), VEC3(4.94, 2.85, -1.16), VEC3(1, .25, 1)));
  triangles[24].uv1 = VEC2(0, 1);
  triangles[24].uv2 = VEC2(1, 1);
  triangles[24].uv3 = VEC2(1, 0);
  triangles[25].uv1 = VEC2(1, 0);
  triangles[25].uv2 = VEC2(0, 0);
  triangles[25].uv3 = VEC2(0, 1); 

  //sd3
  triangles.push_back(Triangle(VEC3(4.941, 1.68, -3.78), VEC3(4.941, 1.68, -1.98), VEC3(4.941, 4.08, -1.98), VEC3(1, .25, 1)));
  triangles.push_back(Triangle(VEC3(4.941, 4.08, -1.98), VEC3(4.941, 4.08, -3.78), VEC3(4.941, 1.68, -3.78), VEC3(1, .25, 1)));
  triangles[26].uv1 = VEC2(0, 1);
  triangles[26].uv2 = VEC2(1, 1);
  triangles[26].uv3 = VEC2(1, 0);
  triangles[27].uv1 = VEC2(1, 0);
  triangles[27].uv2 = VEC2(0, 0);
  triangles[27].uv3 = VEC2(0, 1); 

  //walls
  triangles.push_back(Triangle(VEC3(4.95, 0, -6), VEC3(4.95, 0, 6), VEC3(4.95, 3, 6), VEC3(.439, .525, .620)));
  triangles.push_back(Triangle(VEC3(4.95, 3, 6), VEC3(4.95, 3, -6), VEC3(4.95, 0, -6), VEC3(.439, .525, .620)));
  triangles.push_back(Triangle(VEC3(4.95, 3, -6), VEC3(4.95, 3, 6), VEC3(4.95, 7.5, 6), VEC3(.529, .596, .624)));
  triangles.push_back(Triangle(VEC3(4.95, 7.5, 6), VEC3(4.95, 7.5, -6), VEC3(4.95, 3, -6), VEC3(.529, .596, .624)));

  triangles.push_back(Triangle(VEC3(4.95, 0, 6), VEC3(-4.05, 0, 6), VEC3(-4.05, 3, 6), VEC3(.439, .525, .620)));
  triangles.push_back(Triangle(VEC3(-4.05, 3, 6), VEC3(4.95, 3, 6), VEC3(4.95, 0, 6), VEC3(.439, .525, .620)));
  triangles.push_back(Triangle(VEC3(4.95, 3, 6), VEC3(-4.05, 3, 6), VEC3(-4.05, 7.5, 6), VEC3(.529, .596, .624)));
  triangles.push_back(Triangle(VEC3(-4.05, 7.5, 6), VEC3(4.95, 7.5, 6), VEC3(4.95, 3, 6), VEC3(.529, .596, .624)));

  //door
  triangles.push_back(Triangle(VEC3(2.5, 0, -6), VEC3(4.05, 0, -6), VEC3(2.5, 2.55, -6), VEC3(.329, .227, .169)));
  triangles.push_back(Triangle(VEC3(4.05, 2.55, -6), VEC3(2.5, 2.55, -6), VEC3(4.05, 0, -6), VEC3(.329, .227, .169)));

  //door wall
  triangles.push_back(Triangle(VEC3(-4.05, 0, -6), VEC3(2.5, 0, -6), VEC3(-4.05, 3, -6), VEC3(.439, .525, .620)));
  triangles.push_back(Triangle(VEC3(2.5, 3, -6), VEC3(-4.05, 3, -6), VEC3(2.5, 0, -6), VEC3(.439, .525, .620)));
  triangles.push_back(Triangle(VEC3(4.05, 0, -6), VEC3(4.95, 0, -6), VEC3(4.05, 3, -6), VEC3(.439, .525, .620)));
  triangles.push_back(Triangle(VEC3(4.95, 3, -6), VEC3(4.05, 3, -6), VEC3(4.95, 0, -6), VEC3(.439, .525, .620)));
  triangles.push_back(Triangle(VEC3(2.5, 2.55, -6), VEC3(4.05, 2.55, -6), VEC3(2.5, 3, -6), VEC3(.439, .525, .620)));
  triangles.push_back(Triangle(VEC3(4.05, 3, -6), VEC3(2.5, 3, -6), VEC3(4.05, 2.55, -6), VEC3(.439, .525, .620)));
  triangles.push_back(Triangle(VEC3(-4.05, 3, -6), VEC3(4.95, 3, -6), VEC3(-4.05, 7.5, -6), VEC3(.529, .596, .624)));
  triangles.push_back(Triangle(VEC3(4.95, 7.5, -6), VEC3(-4.05, 7.5, -6), VEC3(4.95, 3, -6), VEC3(.529, .596, .624)));

  //paint lines
  triangles.push_back(Triangle(VEC3(2.45, .000001, -2.289), VEC3(2.45, .000001, -2.369), VEC3(-1.55, .000001, -2.369), VEC3(1, 1, 1)));
  triangles.push_back(Triangle(VEC3(-1.55, .000001, -2.369), VEC3(-1.55, .000001, -2.289), VEC3(2.45, .000001, -2.289), VEC3(1, 1, 1)));
  triangles.push_back(Triangle(VEC3(2.45, .000001, -2.289), VEC3(2.45, .000001, -2.369), VEC3(-1.55, .000001, -2.369), VEC3(1, 1, 1)));
  triangles.push_back(Triangle(VEC3(-1.55, .000001, -2.369), VEC3(-1.55, .000001, -2.289), VEC3(2.45, .000001, -2.289), VEC3(1, 1, 1)));
  triangles.push_back(Triangle(VEC3(2.45, .000001, -2.329), VEC3(2.37, .000001, -2.329), VEC3(2.37, .000001, 6), VEC3(1, 1, 1)));
  triangles.push_back(Triangle(VEC3(2.37, .000001, 6), VEC3(2.45, .000001, 6), VEC3(2.45, .000001, -2.329), VEC3(1, 1, 1)));
  triangles.push_back(Triangle(VEC3(-1.47, .000001, -2.329), VEC3(-1.55, .000001, -2.329), VEC3(-1.55, .000001, 6), VEC3(1, 1, 1)));
  triangles.push_back(Triangle(VEC3(-1.55, .000001, 6), VEC3(-1.47, .000001, 6), VEC3(-1.47, .000001, -2.329), VEC3(1, 1, 1)));
  triangles.push_back(Triangle(VEC3(4.95, .000001, 6), VEC3(4.95, .000001, 5.92), VEC3(-4.05, .000001, 5.92), VEC3(1, 1, 1)));
  triangles.push_back(Triangle(VEC3(-4.05, .000001, 5.92), VEC3(-4.05, .000001, 6), VEC3(4.95, .000001, 6), VEC3(1, 1, 1)));

  // backboard paint
  triangles.push_back(Triangle(VEC3(.05, 2.73, 2.9999), VEC3(.85, 2.73, 2.9999), VEC3(.85, 2.65, 2.9999), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(.85, 2.65, 2.9999), VEC3(.05, 2.65, 2.9999), VEC3(.05, 2.73, 2.9999), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(.05, 3.18, 2.9999), VEC3(.85, 3.18, 2.9999), VEC3(.85, 3.1, 2.9999), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(.85, 3.1, 2.9999), VEC3(.05, 3.1, 2.9999), VEC3(.05, 3.18, 2.9999), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(.77, 3.1, 2.9999), VEC3(.85, 3.1, 2.9999), VEC3(.85, 2.73, 2.9999), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(.85, 2.73, 2.9999), VEC3(.77, 2.73, 2.9999), VEC3(.77, 3.1, 2.9999), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(.05, 3.1, 2.9999), VEC3(.13, 3.1, 2.9999), VEC3(.13, 2.73, 2.9999), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(.13, 2.73, 2.9999), VEC3(.05, 2.73, 2.9999), VEC3(.05, 3.1, 2.9999), VEC3(.851, .333, .012)));

  //backboard
  triangles.push_back(Triangle(VEC3(1.37, 3.52, 3), VEC3(-.47, 2.58, 3), VEC3(-.47, 3.52, 3), VEC3(1, 1, 1)));
  triangles.push_back(Triangle(VEC3(1.37, 2.58, 3), VEC3(-.47, 2.58, 3), VEC3(1.37, 3.52, 3), VEC3(1, 1, 1)));
  //backboard back
  triangles.push_back(Triangle(VEC3(1.45, 3.6, 3.001), VEC3(-.55, 2.5, 3.001), VEC3(-.55, 3.6, 3.001), VEC3(1, 1, 1)));
  triangles.push_back(Triangle(VEC3(1.45, 2.5, 3.001), VEC3(-.55, 2.5, 3.001), VEC3(1.45, 3.6, 3.001), VEC3(1, 1, 1)));

  //backboard inner box paint
  triangles.push_back(Triangle(VEC3(-.55, 3.6, 3), VEC3(1.45, 3.6, 3), VEC3(1.45, 3.52, 3), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(1.45, 3.52, 3), VEC3(-.55, 3.52, 3), VEC3(-.55, 3.6, 3), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(-.55, 2.58, 3), VEC3(1.45, 2.58, 3), VEC3(1.45, 2.5, 3), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(1.45, 2.5, 3), VEC3(-.55, 2.5, 3), VEC3(-.55, 2.58, 3), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(1.37, 3.52, 3), VEC3(1.45, 3.52, 3), VEC3(1.45, 2.58, 3), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(1.45, 2.58, 3), VEC3(1.37, 2.58, 3), VEC3(1.37, 3.52, 3), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(-.55, 3.52, 3), VEC3(-.47, 3.52, 3), VEC3(-.47, 2.58, 3), VEC3(.851, .333, .012)));
  triangles.push_back(Triangle(VEC3(-.47, 2.58, 3), VEC3(-.55, 2.58, 3), VEC3(-.55, 3.52, 3), VEC3(.851, .333, .012)));


  //hoop
  VEC3 hoopCenter(0.45, 2.6, 2.625);
  float radius = .375;
  int sides = 12;
  float rimWidth = 0.05;

  for(int i = 0; i < sides; i++){
    float angle = (2 * M_PI / sides);
    float p1x = hoopCenter[0] + sin(angle * i) * radius;
    float p1z = hoopCenter[2] + cos(angle * i) * radius;
    float p2x = hoopCenter[0] + sin(angle * (i + 1)) * radius;
    float p2z = hoopCenter[2] + cos(angle * (i + 1)) * radius;

    if(i < 4 or i > 8){
      triangles.push_back(Triangle(VEC3(p2x, hoopCenter[1], p2z), VEC3(p1x, hoopCenter[1], p1z), VEC3(p2x, hoopCenter[1] + rimWidth, p2z), VEC3(.851, .333, .012)));
      triangles.push_back(Triangle(VEC3(p2x, hoopCenter[1] + rimWidth, p2z), VEC3(p1x, hoopCenter[1], p1z), VEC3(p1x, hoopCenter[1] + rimWidth, p1z), VEC3(.851, .333, .012)));
    }
    else{
      triangles.push_back(Triangle(VEC3(p1x, hoopCenter[1], p1z), VEC3(p2x, hoopCenter[1], p2z), VEC3(p2x, hoopCenter[1] + rimWidth, p2z), VEC3(.851, .333, .012)));
      triangles.push_back(Triangle(VEC3(p1x, hoopCenter[1], p1z), VEC3(p2x, hoopCenter[1] + rimWidth, p2z), VEC3(p1x, hoopCenter[1] + rimWidth, p1z), VEC3(.851, .333, .012)));
    }
  }

  //lights
  lights.push_back(Light(VEC3(2.75, 8.5, 0), VEC3(.702, .682, .588), 2, 1));
  lights.push_back(Light(VEC3(-1.85, 8.5, 0), VEC3(.702, .682, .588), 2, 1));
}

//////////////////////////////////////////////////////////////////////////////////
//Still need some tweaks to certain frames to make it smoother
//////////////////////////////////////////////////////////////////////////////////
void getBallPos(){
  ballPos.push_back(VEC3(0.22, 1.08, -1.54)); //frame 0
  ballPos.push_back(VEC3(0.21, 1.075, -1.53)); //frame 10
  ballPos.push_back(VEC3(0.20, 1.073, -1.52)); //frame 20
  ballPos.push_back(VEC3(0.151, 1.05, -1.47)); //frame 30
  ballPos.push_back(VEC3(0.04, 1.18, -1.45)); //frame 40
  ballPos.push_back(VEC3(0.039, 0.99, -1.43)); //frame 50
  ballPos.push_back(VEC3(0.13, 0.53, -1.4)); //frame 60
  ballPos.push_back(VEC3(0.23, .125, -1.37)); //frame 70
  ballPos.push_back(VEC3(0.35, .5, -1.3)); //frame 80
  ballPos.push_back(VEC3(0.43, 1.1, -1.232)); //frame 90
  ballPos.push_back(VEC3(0.435, 1.07, -1.22)); //frame 93
  ballPos.push_back(VEC3(0.43, 1.04, -1.204)); //frame 96
  ballPos.push_back(VEC3(0.370, .93, -1.145)); //frame 100
  ballPos.push_back(VEC3(0.18, 0.56, -1)); //frame 110
  ballPos.push_back(VEC3(0.01, .125, -0.78)); //frame 120
  ballPos.push_back(VEC3(-0.11, 0.69, -0.72)); //frame 130
  ballPos.push_back(VEC3(-0.137, 0.58, -0.62)); //frame 133
  ballPos.push_back(VEC3(-0.172, 0.43, -0.52)); //frame 136
  ballPos.push_back(VEC3(-0.253, 0.125, -0.425)); //frame 140
  ballPos.push_back(VEC3(-0.13, 0.725, -0.070)); //frame 150
  ballPos.push_back(VEC3(-0.125, 0.894, 0.52)); //frame 160
  ballPos.push_back(VEC3(0.195, 0.844, 1.05)); //frame 170
  ballPos.push_back(VEC3(0.235, 0.875, 1.24)); //frame 173
  ballPos.push_back(VEC3(0.31, 1.024, 1.47)); //frame 176
  ballPos.push_back(VEC3(0.405, 1.52, 1.6)); //frame 180
  ballPos.push_back(VEC3(0.31, 2.07, 1.71)); //frame 186
  ballPos.push_back(VEC3(0.169, 2.35, 1.75)); //frame 190
  ballPos.push_back(VEC3(0.055, 2.57, 1.8)); //frame 195
  ballPos.push_back(VEC3(0.09, 2.72, 1.86)); //frame 200
  ballPos.push_back(VEC3(0.17, 2.8, 2.05)); //frame 205
  ballPos.push_back(VEC3(0.3, 2.78, 2.26)); //frame 210
  ballPos.push_back(VEC3(0.45, 2.6, 2.625)); //frame 213
  ballPos.push_back(VEC3(0.6, .125, 3.2)); //frame 220
  ballPos.push_back(VEC3(0.487, .78, 2.93)); //frame 230
  ballPos.push_back(VEC3(0.41, 1.4, 2.84)); //frame 240
  ballPos.push_back(VEC3(0.133, 1.9, 2.72)); //frame 250
  ballPos.push_back(VEC3(0.02, 1.5, 2.65)); //frame 260
  ballPos.push_back(VEC3(-0.069, 1.291, 2.673)); //frame 270
  ballPos.push_back(VEC3(-0.16, 1.261, 2.67)); //frame 280
  ballPos.push_back(VEC3(-0.172, 1.254, 2.66)); //frame 290
  ballPos.push_back(VEC3(-0.109, 1.252, 2.671)); //frame 300

}


vector<int> keyFrames = {
        0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 93, 96,
        100, 110, 120, 130, 133, 136, 140, 150, 160, 170, 
        173, 176, 180, 186, 190, 195, 200, 205, 210, 213, 220, 230, 240, 250, 
        260, 270, 280, 290, 299
    };


// Linear interpolation between two points
VEC3 interpolate(const VEC3& p0, const VEC3& p1, float alpha) {
    return (1 - alpha) * p0 + alpha * p1;
}


int findSegment(int currFrame) {
    int segment = 0;
    for (int i = 0; i < keyFrames.size() - 1; i++) {
        if (currFrame >= keyFrames[i] && currFrame < keyFrames[i + 1]) {
            segment = i;
            break;
        }
    }
    return segment;
}




//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  
  string skeletonFilename("basketball.asf");
  string motionFilename("basketball.amc");

  // load up skeleton stuff
  skeleton = new Skeleton(skeletonFilename.c_str(), MOCAP_SCALE);
  skeleton->setBasePosture();
  displayer.LoadSkeleton(skeleton);

  // load up the motion
  motion = new Motion(motionFilename.c_str(), MOCAP_SCALE, skeleton);
  displayer.LoadMotion(motion);
  skeleton->setPosture(*(displayer.GetSkeletonMotion(0)->GetPosture(0)));

  getBallPos();
  int framesPerSegment = 300 / (ballPos.size() - 1);

  //By separating the static scenes we save time rendering only once
  buildStaticScenes();

  readPPM("textures/basketball_floor.ppm", floorWidth, floorHeight, floorTexture);
  readPPM("textures/Young_Bron.ppm", youngBronWidth, youngBronHeight, youngBron);
  readPPM("textures/The_Chosen_One.ppm", chosenBronWidth, chosenBronHeight, chosenBron);
  readPPM("textures/Slam_Dunk_1.ppm", SD1Width, SD1Height, SD1);
  readPPM("textures/Slam_Dunk_2.ppm", SD2Width, SD2Height, SD2);
  readPPM("textures/Slam_Dunk_3.ppm", SD3Width, SD3Height, SD3);

  for (int x = 166; x < 600; x += 2)
  {
    setSkeletonsToSpecifiedFrame(x);
    buildScene();
 
    int currFrame = x / 2;
    int segment = findSegment(currFrame);

    int startFrame = keyFrames[segment];
    int endFrame = keyFrames[segment + 1];
    float alpha = (currFrame - startFrame) / (float)(endFrame - startFrame);

    VEC3 currentBallPos = interpolate(ballPos[segment], ballPos[segment + 1], alpha);

    //basketball
    sphereCenters.push_back(currentBallPos);
    sphereRadii.push_back(.125);
    sphereColors.push_back(VEC3(0.965, 0.51, 0.22));
    
    //door knob
    sphereCenters.push_back(VEC3(3.9, 1.16, -5.9));
    sphereRadii.push_back(.1);
    sphereColors.push_back(VEC3(0.0, 1.0, 0.0));    

    //court pattern
    circles.push_back(Circle(VEC3(.45, .000001, -2.329), VEC3(.45, -1, -2.329), VEC3(1,1,1), 2));
    circles.push_back(Circle(VEC3(.45, .000001, 3.815), VEC3(.45, -1, 3.815), VEC3(1,1,1), 8.15));
    
    //backboard support
    cylinder.push_back(Cylinder(VEC3(1, 2.9, 3.001), VEC3(1, 2.9, 6), VEC3(.333, .333, .333), .08));
    cylinder.push_back(Cylinder(VEC3(-.1, 2.9, 3.001), VEC3(-.1, 2.9, 6), VEC3(.333, .333, .333), .08));
    cylinder.push_back(Cylinder(VEC3(1, 3.2, 3.001), VEC3(1, 3.2, 6), VEC3(.333, .333, .333), .08));
    cylinder.push_back(Cylinder(VEC3(-.1, 3.2, 3.001), VEC3(-.1, 3.2, 6), VEC3(.333, .333, .333), .08));

    

    // Have camera track the ball
    lookingAt = currentBallPos;

    //Have camera move in spiral 
    float theta = x * 0.0005f; //rotation increment
    float radius = 5.0f;            //radius from ball
    float height = 1.08f + x * 0.005f; //height increment

    eye = VEC3(
      -(lookingAt[0] + radius * cos(theta)), //X 
      height,                                //Y
      lookingAt[2] + radius * sin(theta)     //Z 
    ); 


    // lookingAt = VEC3(.45, 3, 5);
    // eye = VEC3(.45, 4, -5);

    // lookingAt = VEC3(4.95, 3, 3);
    // eye = VEC3(-4.05, 3, 3);

    char buffer[256];
    snprintf(buffer, 256, "./frames/frame.%04i.ppm", x / 2);
    

    renderImage(windowWidth, windowHeight, buffer);
    cout << "Rendered " + to_string(x / 2) + " frames" << endl;
  }

  return 0;
}
