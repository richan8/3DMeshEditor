// OpenGL Helpers to reduce the clutter
#include "Helpers.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>
#else
// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>
#endif

// Linear Algebra Library
#include <Eigen/Core>

// Timer
#include <chrono>

// VertexBufferObject wrapper
VertexBufferObject VBO;
VertexBufferObject VBOColor;
VertexBufferObject VBOVFlatNormals;


// Contains the vertex positions and colors.
Eigen::MatrixXf V(4,0);
Eigen::MatrixXf C(4,0);
Eigen::MatrixXf Centroids(3,0);
Eigen::MatrixXf VFlatNormals(3,0);


Eigen::MatrixXf VLines(4,0);
Eigen::MatrixXf CLines(4,0);
Eigen::MatrixXf LineCentroids(3,0);

Eigen::MatrixXf VCursorLine(4,2);
Eigen::MatrixXf CCursorLine(4,2);
Eigen::MatrixXf CursorTemp(3,2);

Eigen::MatrixXi ObjVIndexes(2,0);
Eigen::MatrixXi ObjLIndexes(2,0);

Eigen::MatrixXf selectedVertexIDandCols(5,0);

// Global Variables
int state = 0;

double X1, Y1, X2, Y2, X3, Y3, dragX1, dragY1, viewScale = 1.0, viewX = 0.0, viewY = 0.0, viewZ = 0.0, camPosX = 0.0;
double camPosY = 0.0, camPosZ = 0.0, camFX = 0.0, camFY= 0.0, camFZ= 1.0, cursorX= 0.0, cursorY= 0.0, cursorZ= -1.0;
double pointX = 0.0, pointY = 0.0, pointZ = 0.0;
int tempTriangleID = -1, closestVertexID = -1, selectedVertexID = -1, selectedIndex = -1, selectStart = -1, selectEnd = -1, LSelectStart = -1, LSelectEnd = 1;
bool mouseDown = false, dynamicLight = false;
float timeGlobal, timeAnimStart;

void framebuffer_size_callback(GLFWwindow* window, int width, int height){
    glViewport(0, 0, width, height);
}

Eigen::MatrixXf triangleNormal(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3){
    Eigen::MatrixXf N(3,1);
    Eigen::MatrixXf U2(3,1);
    Eigen::MatrixXf V2(3,1);
    U2.col(0) << x2-x1, y2-y1, z2-z1;
    V2.col(0) << x3-x1, y3-y1, z3-z1;
    N.col(0) << U2.coeff(1,0)*V2.coeff(2,0) - U2.coeff(2,0)*V2.coeff(1,0)
              , U2.coeff(2,0)*V2.coeff(0,0) - U2.coeff(0,0)*V2.coeff(2,0)
              , U2.coeff(0,0)*V2.coeff(1,0) - U2.coeff(1,0)*V2.coeff(0,0);
    return(N);
}

bool vertexInTriangle(int vertexID, int TriangleID){
    if((V.coeff(1,vertexID) == V.coeff(1,TriangleID)) && (V.coeff(2,vertexID) == V.coeff(2,TriangleID)) && (V.coeff(3,vertexID) == V.coeff(3,TriangleID))){
        return(true);
    }
    else if((V.coeff(1,vertexID) == V.coeff(1,TriangleID+1)) && (V.coeff(2,vertexID) == V.coeff(2,TriangleID+1)) && (V.coeff(3,vertexID) == V.coeff(3,TriangleID+1))){
        return(true);
    }
    else if((V.coeff(1,vertexID) == V.coeff(1,TriangleID+2)) && (V.coeff(2,vertexID) == V.coeff(2,TriangleID+2)) && (V.coeff(3,vertexID) == V.coeff(3,TriangleID+2))){
        return(true);
    }
    else{
        return(false);
    }
}

void setFlatNormals(int iterations, double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3){
    Eigen::MatrixXf N = triangleNormal(x1, y1, z1, x2, y2, z2, x3, y3, z3);
    int n = VFlatNormals.cols();
    int i;
    for(i = n; i < n + iterations; i++){
        VFlatNormals.conservativeResize(3,VFlatNormals.cols()+1);
        VFlatNormals.col(i) << N.coeff(0,0), N.coeff(1,0), N.coeff(2,0);
    }
}

void drawLines(double x1, double y1, double z1, double x2, double y2, double z2){
    int i = VLines.cols();

    VLines.conservativeResize(4,i+2);
    CLines.conservativeResize(4,i+2);

    VLines.col(i+0) << i,x1,y1,z1;
    VLines.col(i+1) << i+1,x2,y2,z2;

    CLines.col(i+0) << 0.1, 0.1, 0.1, 1.0;
    CLines.col(i+1) << 0.1, 0.1, 0.1 ,1.0;

    LineCentroids.conservativeResize(3,i+3);
    LineCentroids.col(i+0) << 0.0, 0.0, 0.0;
    LineCentroids.col(i+1) << 0.0, 0.0, 0.0;
    LineCentroids.col(i+2) << 0.0, 0.0, 0.0;
}

void drawTriangle(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3){
    int i = V.cols();
    V.conservativeResize(4,i+3);
    C.conservativeResize(4,i+3);

    V.col(i+0) << i,x1,y1,z1;
    V.col(i+1) << i+1,x2,y2,z2;
    V.col(i+2) << i+2,x3,y3,z3;

    drawLines(x1, y1, z1, x2, y2, z2);
    drawLines(x2, y2, z2, x3, y3, z3);
    drawLines(x3, y3, z3, x1, y1, z1);

    C.col(i+0) << 0.95, 0.95, 0.95, 0.8;
    C.col(i+1) << 0.95, 0.95, 0.95, 0.8;
    C.col(i+2) << 0.95, 0.95, 0.95, 0.8;

    Centroids.conservativeResize(3,i+3);
    Centroids.col(i+0) << 0.0, 0.0, 0.0;
    Centroids.col(i+1) << 0.0, 0.0, 0.0;
    Centroids.col(i+2) << 0.0, 0.0, 0.0;

    setFlatNormals(3, x1, y1, z1, x2, y2, z2, x3, y3, z3);
}

float getSquaredDistance(double x1, double y1, double z1, double x2, double y2, double z2){
    return(((x2-x1)*(x2-x1)) + ((y2-y1)*(y2-y1)) + ((z2-z1)*(z2-z1)));
}

bool pointInsideTriangle(double xLocal1, double yLocal1, double xLocal2, double yLocal2, double xLocal3, double yLocal3, double xLocalPoint, double yLocalPoint){
    //Solving by using barycentric co-ordinates
    //Attribution: http://totologic.blogspot.com/2014/01/accurate-point-in-triangle-test.html
    double a = ((yLocal2 - yLocal3)*(xLocalPoint - xLocal3) + (xLocal3 - xLocal2)*(yLocalPoint - yLocal3)) / ((yLocal2 - yLocal3)*(xLocal1 - xLocal3) + (xLocal3 - xLocal2)*(yLocal1 - yLocal3));
    double b = ((yLocal3 - yLocal1)*(xLocalPoint - xLocal3) + (xLocal1 - xLocal3)*(yLocalPoint - yLocal3)) / ((yLocal2 - yLocal3)*(xLocal1 - xLocal3) + (xLocal3 - xLocal2)*(yLocal1 - yLocal3));
    double c = 1 - a - b;
    return(a>=0 && a<=1 && b>=0 && b<=1 && c>=0 && c<=1);
}

Eigen::MatrixXf scaleAndCenter(Eigen::MatrixXf M){
    Eigen::MatrixXf res(3,0);
    int i;
    float x,y,z;
    float xAgg = 0;
    float yAgg = 0;
    float zAgg = 0;

    float xMax = 0., yMax = 0., zMax = 0.;
    float xVal, yVal, zVal;
    for(i=0; i<M.cols(); i++){
        xVal = M.coeff(0,i);
        yVal = M.coeff(1,i);
        zVal = M.coeff(2,i);
        
        xMax = abs(xVal) > abs(xMax) ? abs(xVal):abs(xMax);
        yMax = abs(yVal) > abs(yMax) ? abs(yVal):abs(yMax);
        zMax = abs(zVal) > abs(zMax) ? abs(zVal):abs(zMax);
    }
    float maxAll = xMax > yMax ? (xMax > zMax) ? xMax:zMax : (yMax > zMax) ? yMax : zMax;

    res.conservativeResize(3,M.cols());
    for(i=0;i<M.cols();i++){
        x = (M.coeff(0,i))/(2*maxAll);
        y = (M.coeff(1,i))/(2*maxAll);
        z = (M.coeff(2,i))/(2*maxAll);
        xAgg += x;
        yAgg += y;
        zAgg += z;
        res.col(i) << x, y ,z;
    }

    for(i=0;i<M.cols();i++){
        x = res.coeff(0,i);
        res.col(i) << (M.coeff(0,i))/(2*maxAll), (M.coeff(1,i))/(2*maxAll), (M.coeff(2,i))/(2*maxAll);
    }

    x = xAgg/(float)res.cols();
    y = yAgg/(float)res.cols();
    z = zAgg/(float)res.cols();

    for(i=0; i<res.cols(); i++){
        res.col(i) << res.coeff(0,i)-x, res.coeff(1,i)-y, res.coeff(2,i)-z;
    }

    return res;
}

void RotatePointX(int i, double angle, boolean line){
    if(i == -1){return;}
    Eigen::Matrix4f pos, transform, newPos;
    double x, y, z, cx, cy, cz, fnX = 0.0, fnY = 0.0, fnZ=0.0;
    if(line){
        cx = LineCentroids.coeff(0,i);
        cy = LineCentroids.coeff(1,i);
        cz = LineCentroids.coeff(2,i);
        x = VLines.coeff(1,i)-cx;
        y = VLines.coeff(2,i)-cy;
        z = VLines.coeff(3,i)-cz;
    }
    else{
        cx = Centroids.coeff(0,i);
        cy = Centroids.coeff(1,i);
        cz = Centroids.coeff(2,i);
        x = V.coeff(1,i)-cx;
        y = V.coeff(2,i)-cy;
        z = V.coeff(3,i)-cz;
        fnX = VFlatNormals.coeff(0,i);
        fnY = VFlatNormals.coeff(1,i);
        fnZ = VFlatNormals.coeff(2,i);
    }

    pos << x, fnX, 0.0, 0.0,
           y, fnY, 0.0, 0.0,
           z, fnZ, 0.0, 0.0,
           1.0, 0.0, 0.0, 0.0;

    transform << 1.0, 0.0, 0.0, 0.0,
                0.0, cos(angle), -sin(angle), 0.0,
                0.0, sin(angle), cos(angle), 0.0,
                0.0, 0.0, 0.0, 1.0;

    newPos = transform * pos;
    if(line){VLines.col(i) << i, newPos.coeff(0,0)+cx, newPos.coeff(1,0)+cy, newPos.coeff(2,0)+cz; }
    else{
        V.col(i) << i, newPos.coeff(0,0)+cx, newPos.coeff(1,0)+cy, newPos.coeff(2,0)+cz;
        VFlatNormals.col(i) << newPos.coeff(0,1), newPos.coeff(1,1), newPos.coeff(2,1);
    }
}

void RotatePointY(int i, double angle, boolean line){
    if(i == -1){return;}
    Eigen::Matrix4f pos, transform, newPos;
    double x, y, z, cx, cy, cz, fnX = 0.0, fnY = 0.0, fnZ=0.0;
    if(line){
        cx = LineCentroids.coeff(0,i);
        cy = LineCentroids.coeff(1,i);
        cz = LineCentroids.coeff(2,i);
        x = VLines.coeff(1,i)-cx;
        y = VLines.coeff(2,i)-cy;
        z = VLines.coeff(3,i)-cz;
    }
    else{
        cx = Centroids.coeff(0,i);
        cy = Centroids.coeff(1,i);
        cz = Centroids.coeff(2,i);
        x = V.coeff(1,i)-cx;
        y = V.coeff(2,i)-cy;
        z = V.coeff(3,i)-cz;
        fnX = VFlatNormals.coeff(0,i);
        fnY = VFlatNormals.coeff(1,i);
        fnZ = VFlatNormals.coeff(2,i);
    }
    pos << x, fnX, 0.0, 0.0,
           y, fnY, 0.0, 0.0,
           z, fnZ, 0.0, 0.0,
           1.0, 0.0, 0.0, 0.0;

    transform << cos(angle), 0.0, sin(angle), 0.0,
                0.0, 1.0, 0.0, 0.0,
                -sin(angle), 0.0, cos(angle), 0.0,
                0.0, 0.0, 0.0, 1.0;

    newPos = transform * pos;
    if(line){VLines.col(i) << i, newPos.coeff(0,0)+cx, newPos.coeff(1,0)+cy, newPos.coeff(2,0)+cz; }
    else{
        V.col(i) << i, newPos.coeff(0,0)+cx, newPos.coeff(1,0)+cy, newPos.coeff(2,0)+cz;
        VFlatNormals.col(i) << newPos.coeff(0,1), newPos.coeff(1,1), newPos.coeff(2,1);
    }
}

void RotatePointZ(int i, double angle, boolean line){
    if(i == -1){return;}
    Eigen::Matrix4f pos, transform, newPos;
    double x, y, z, cx, cy, cz, fnX = 0.0, fnY = 0.0, fnZ=0.0;
    if(line){
        cx = LineCentroids.coeff(0,i);
        cy = LineCentroids.coeff(1,i);
        cz = LineCentroids.coeff(2,i);
        x = VLines.coeff(1,i)-cx;
        y = VLines.coeff(2,i)-cy;
        z = VLines.coeff(3,i)-cz;
    }
    else{
        cx = Centroids.coeff(0,i);
        cy = Centroids.coeff(1,i);
        cz = Centroids.coeff(2,i);
        x = V.coeff(1,i)-cx;
        y = V.coeff(2,i)-cy;
        z = V.coeff(3,i)-cz;
        fnX = VFlatNormals.coeff(0,i);
        fnY = VFlatNormals.coeff(1,i);
        fnZ = VFlatNormals.coeff(2,i);
    }
    pos << x, fnX, 0.0, 0.0,
           y, fnY, 0.0, 0.0,
           z, fnZ, 0.0, 0.0,
           1.0, 0.0, 0.0, 0.0;

    transform << cos(angle), -sin(angle), 0.0, 0.0,
                sin(angle), cos(angle), 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;

    newPos = transform * pos;
    if(line){VLines.col(i) << i, newPos.coeff(0,0)+cx, newPos.coeff(1,0)+cy, newPos.coeff(2,0)+cz; }
    else{
        V.col(i) << i, newPos.coeff(0,0)+cx, newPos.coeff(1,0)+cy, newPos.coeff(2,0)+cz;
        VFlatNormals.col(i) << newPos.coeff(0,1), newPos.coeff(1,1), newPos.coeff(2,1);
    }
}

void ScalePoint(int i, double scale, boolean line){
    if(i == -1){return;}
    Eigen::Matrix4f pos, transform, newPos;
    double x, y, z, cx, cy, cz;
    if(line){
        cx = LineCentroids.coeff(0,i);
        cy = LineCentroids.coeff(1,i);
        cz = LineCentroids.coeff(2,i);
        x = VLines.coeff(1,i)-cx;
        y = VLines.coeff(2,i)-cy;
        z = VLines.coeff(3,i)-cz;
    }
    else{
        cx = Centroids.coeff(0,i);
        cy = Centroids.coeff(1,i);
        cz = Centroids.coeff(2,i);
        x = V.coeff(1,i)-cx;
        y = V.coeff(2,i)-cy;
        z = V.coeff(3,i)-cz;
    }
    pos << x, 0.0, 0.0, 0.0,
           y, 0.0, 0.0, 0.0,
           z, 0.0, 0.0, 0.0,
           1.0, 1.0, 1.0, 1.0;

    transform << scale, 0.0, 0.0, 0.0,
                0.0, scale, 0.0, 0.0,
                0.0, 0.0, scale, 0.0,
                0.0, 0.0, 0.0, 1.0;

    newPos = transform * pos;
    if(line){VLines.col(i) << i, newPos.coeff(0,0)+cx, newPos.coeff(1,0)+cy, newPos.coeff(2,0)+cz; }
    else{V.col(i) << i, newPos.coeff(0,0)+cx, newPos.coeff(1,0)+cy, newPos.coeff(2,0)+cz; }
}

void TranslatePoint(int i, double x, double y, double z, boolean line){
    if(i == -1){return;}
    if(line){
        VLines.col(i) << VLines.coeff(0,i), VLines.coeff(1,i)+x, VLines.coeff(2,i)+y, VLines.coeff(3,i)+z;
        LineCentroids.col(i) << LineCentroids.coeff(0,i)+x, LineCentroids.coeff(1,i)+y, LineCentroids.coeff(2,i)+z;
    }
    else{
        V.col(i) << V.coeff(0,i), V.coeff(1,i)+x, V.coeff(2,i)+y, V.coeff(3,i)+z;
        Centroids.col(i) << Centroids.coeff(0,i)+x, Centroids.coeff(1,i)+y, Centroids.coeff(2,i)+z;
    }
}

void rotateObject(int objectID, double a1, double a2, double a3){
    if(objectID == -1){return;}
    int v1 = ObjVIndexes.coeff(0,objectID);
    int v2 = ObjVIndexes.coeff(1,objectID);
    int Lv1 = ObjLIndexes.coeff(0,objectID);
    int Lv2 = ObjLIndexes.coeff(1,objectID);
    int i;
    if(a1 != 0.0){
        for(i = v1; i < v2; i++){RotatePointX(i, a1, false);}
        for(i = Lv1; i < Lv2; i++){RotatePointX(i, a1, true);}
    }
    if(a2 != 0.0){
        for(i = v1; i < v2; i++){RotatePointY(i, a2, false);}
        for(i = Lv1; i < Lv2; i++){RotatePointY(i, a2, true);}
    }
    if(a3 != 0.0){
        for(i = v1; i < v2; i++){RotatePointZ(i, a3, false);}
        for(i = Lv1; i < Lv2; i++){RotatePointZ(i, a3, true);}
    }
}

void scaleObject(int objectID, double scale){
    if(objectID == -1){return;}
    int v1 = ObjVIndexes.coeff(0,objectID);
    int v2 = ObjVIndexes.coeff(1,objectID);
    int Lv1 = ObjLIndexes.coeff(0,objectID);
    int Lv2 = ObjLIndexes.coeff(1,objectID);
    int i;
    for(i = v1; i < v2; i++){ScalePoint(i, scale, false);}
    for(i = Lv1; i < Lv2; i++){ScalePoint(i, scale, true);}
}

void translateObject(int objectID, double x, double y, double z){
    if(objectID == -1){return;}
    int v1 = ObjVIndexes.coeff(0,objectID);
    int v2 = ObjVIndexes.coeff(1,objectID);
    int Lv1 = ObjLIndexes.coeff(0,objectID);
    int Lv2 = ObjLIndexes.coeff(1,objectID);
    int i;
    for(i = v1; i < v2; i++){TranslatePoint(i, x, y, z, false);}
    for(i = Lv1; i < Lv2; i++){TranslatePoint(i, x, y, z, true);}
}


///// LOAD MESH
void loadMesh(int meshID){
    std::ifstream meshFile;
    meshFile.open("../data/mesh"+std::to_string(meshID)+".off");
    //meshFile.open("../data/bunny.off");
    std::string line;
    getline(meshFile,line);
    getline(meshFile,line);
    std::istringstream ss(line);
    int i = 0, N1 = 0, N2 = 0;
    while(ss){
        std::string num;
        ss >> num;
        if(i==0){N1 = stoi(num);}
        if(i==1){N2 = stoi(num);}
        i+=1;
    }

    meshFile.close();
    Eigen::MatrixXf VTemp1(N1,3);
    Eigen::MatrixXi VTemp2(N2,3);
    meshFile.open("../data/mesh"+std::to_string(meshID)+".off");
    //meshFile.open("../data/bunny.off");
    i = 1;
    int j = 0;
    int count = 0;
    if(meshFile.is_open()){
        while(!meshFile.eof()){
            std::string line;
            getline(meshFile,line);
            std::istringstream ss(line);
            if(i != 1 && i != 2 && i<=N1+2){
                j = 0;
                do{
                    std::string num;
                    ss >> num;
                    VTemp1(i - 3,j) = stof(num);
                    j++;
                }while(ss && j!=3);
            }
            if(i != 1 && i != 2 && i>N1+2 && i<=N1+N2+2){
                int j=0;
                do{
                    std::string num;
                    ss>>num;
                    if(j>0){
                        VTemp2(i-(N1+3),j-1) = stoi(num);
                    }
                    j++;
                }while(ss && j!=4);
            }
            i++;
        }
    }
    meshFile.close();

    printf("INTERMEDIATE CODE COMPLETE. LENGTH: %i\n",VTemp2.rows());
    Eigen::MatrixXf MeshData(3,0);
    Eigen::MatrixXf MeshFlatNormalsData(3,0);

    for(j=0; j<VTemp2.rows(); j++){
        int i = MeshData.cols();
        MeshData.conservativeResize(3,i+3);
        MeshData.col(i+0) << VTemp1.coeff(VTemp2.coeff(j,0),0), VTemp1.coeff(VTemp2.coeff(j,0),1), VTemp1.coeff(VTemp2.coeff(j,0),2);
        MeshData.col(i+1) << VTemp1.coeff(VTemp2.coeff(j,1),0), VTemp1.coeff(VTemp2.coeff(j,1),1), VTemp1.coeff(VTemp2.coeff(j,1),2);
        MeshData.col(i+2) << VTemp1.coeff(VTemp2.coeff(j,2),0), VTemp1.coeff(VTemp2.coeff(j,2),1), VTemp1.coeff(VTemp2.coeff(j,2),2);
    }

    printf("SCALING AND CENTERING THE MESH\n");
    MeshData = scaleAndCenter(MeshData);
    
    printf("COMPUTING FLAT NORMALS\n");
    Eigen::MatrixXf N(3,1);
    int n;

    MeshFlatNormalsData.conservativeResize(3, MeshData.cols());
    for(i=0; i < MeshData.cols(); i+=3){
        Eigen::MatrixXf N = triangleNormal(MeshData.coeff(0,i+0), MeshData.coeff(1,i+0), MeshData.coeff(2,i+0),
                                           MeshData.coeff(0,i+1), MeshData.coeff(1,i+1), MeshData.coeff(2,i+1),
                                           MeshData.coeff(0,i+2), MeshData.coeff(1,i+2), MeshData.coeff(2,i+2));

        MeshFlatNormalsData.col(i) << N.coeff(0,0), N.coeff(1,0), N.coeff(2,0);
        MeshFlatNormalsData.col(i+1) << N.coeff(0,0), N.coeff(1,0), N.coeff(2,0);
        MeshFlatNormalsData.col(i+2) << N.coeff(0,0), N.coeff(1,0), N.coeff(2,0);
    }

    int v1 = V.cols();
    int Lv1 = VLines.cols();

    for(i = 0; i < MeshData.cols(); i+=3){
        drawTriangle(MeshData.coeff(0,i+0), MeshData.coeff(1,i+0), MeshData.coeff(2,i+0),
                     MeshData.coeff(0,i+1), MeshData.coeff(1,i+1), MeshData.coeff(2,i+1),
                     MeshData.coeff(0,i+2), MeshData.coeff(1,i+2), MeshData.coeff(2,i+2));
    }

    int v2 = V.cols();
    int Lv2 = VLines.cols();

    printf("\nV1,V2, LV1, LV2 -> %i, %i\t %i, %i\n", v1, v2, Lv1, Lv2);

    n = ObjVIndexes.cols();

    ObjVIndexes.conservativeResize(2, n+1);
    ObjLIndexes.conservativeResize(2, n+1);

    selectStart = v1;
    selectEnd = v2;
    LSelectStart = Lv1;
    LSelectEnd = Lv2;

    ObjVIndexes.col(n) << v1, v2;
    ObjLIndexes.col(n) << Lv1, Lv2;

    if(selectedIndex == -1){selectedIndex = 0;}
    
    printf("MESH LOADED SUCCESSFULLY WITH %i VERTEXES\n", MeshData.cols());
    printf("VERTEX MATRIX SIZE: %i\n", V.cols());
    printf("OBJECTS :");
    for(i=0; i < ObjVIndexes.cols(); i++){
        printf("%i, %i\t", ObjVIndexes.coeff(i,0), ObjVIndexes.coeff(i,1));
    }
    printf("\n");
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods){
    // Get the position of the mouse in the window
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Get the size of the window
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // Convert screen position to world coordinates
    double cursorX = ((xpos/double(width))*2)-1-camPosX;
    double cursorY = (((height-1-ypos)/double(height))*2)-1-camPosY;
    
    float newX = 0.0, newY = 0.0, newZ = 0.0;

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS){
        printf("Click at: (%f, %f, %f)\n", cursorX, cursorY, cursorZ);

        int i, j, vertexID = -1;
        float minDistance = -1, distance = -1, zDistance = -1, minZDistance = -1;

        for(i = 0; i < V.cols(); i+=3){
            if(pointInsideTriangle(V.coeff(1,i), V.coeff(2,i), V.coeff(1,i+1), V.coeff(2,i+1), V.coeff(1,i+2), V.coeff(2,i+2), cursorX, cursorY)){
                zDistance = -abs(-1+((V.coeff(3,i)+V.coeff(3,i+1)+V.coeff(3,i+2))/3));
                if(minZDistance == -1 || zDistance < minZDistance){
                    minZDistance = zDistance;
                    minDistance = -1;
                    for(j = 0; j < 3; j++){
                        distance = getSquaredDistance(V.coeff(1,i+j), V.coeff(2,i+j), 0.0, cursorX, cursorY, 0.0);
                        if((minDistance == -1) || (distance < minDistance)){
                            minDistance = distance;
                            vertexID = i+j;
                            pointX = V.coeff(1,vertexID);
                            pointY = V.coeff(2,vertexID);
                            pointZ = V.coeff(3,vertexID);
                        }
                    }
                }
            }
        }
        if(vertexID > -1){
            int i, n;
            selectedVertexID = vertexID;
            if(selectedVertexID != -1){
                for(i=0; i<selectedVertexIDandCols.cols(); i++){
                    C.col(selectedVertexIDandCols.coeff(0,i)) << selectedVertexIDandCols.coeff(1,i), selectedVertexIDandCols.coeff(2,i), selectedVertexIDandCols.coeff(3,i), 0.8;
                }
            }
            selectedVertexIDandCols.conservativeResize(5,0);
            for(i=0; i < V.cols(); i++){
                if((V.coeff(1,selectedVertexID)==V.coeff(1,i)) && (V.coeff(2,selectedVertexID)==V.coeff(2,i)) && (V.coeff(3,selectedVertexID)==V.coeff(3,i))){
                    n = selectedVertexIDandCols.cols();
                    selectedVertexIDandCols.conservativeResize(5,n+1);
                    selectedVertexIDandCols.col(n) << i, C.coeff(0,i), C.coeff(1,i), C.coeff(2,i), C.coeff(3,i);
                    C.col(i) << 0.3, 0.3, 1.0, 1.0;
                }
            }
            printf("SELECTED VERTEX AND DISTANCE: %i\t %f\n", selectedVertexID, minDistance);
            for(i = 0; i < ObjVIndexes.cols(); i++){
                printf("SELECTED VERTEX ID: %i\n", selectedVertexID);
                printf("CHECKING IN RANGE: %i, %i\n", ObjVIndexes.coeff(0, i),ObjVIndexes.coeff(1, i));
                if(selectedVertexID > ObjVIndexes.coeff(0, i) && selectedVertexID < ObjVIndexes.coeff(1, i)){
                    selectedIndex = i;
                    selectStart = ObjVIndexes.coeff(selectedIndex,0);
                    selectEnd = ObjVIndexes.coeff(selectedIndex,1);
                    LSelectStart = ObjLIndexes.coeff(selectedIndex, 0);
                    LSelectEnd = ObjLIndexes.coeff(selectedIndex,1);
                    printf("SELECTED OBJECT : %i\n", selectedIndex);
                    break;
                }
            }
        }
        else{
            printf("NOTHING TO SELECT\n");
        }
    }

    // Upload the change to the GPU
    VBO.update(V);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods){
    int i, iLines, j;
    double xVals, yVals, zVals, alphaVal, distance, minDistance;
    double newXVals, newYVals, newZVals;
    if(action == GLFW_PRESS){
        switch (key){
            case GLFW_KEY_1:
                printf("\nLOADING MESH: KLINGON D7 CLASS BATTLECRUISER\n");
                loadMesh(0);
                break;

            case GLFW_KEY_2:
                printf("\nLOADING MESH: SKULL\n");
                loadMesh(1);
                break;

            case GLFW_KEY_3:
                printf("\nLOADING MESH: SEASHELL\n");
                loadMesh(2);
                break;

            case GLFW_KEY_4:
                printf("\nLOADING MESH: HORSE\n");
                loadMesh(3);
                break;

            case GLFW_KEY_5:
                printf("\nLOADING MESH: DOG\n");
                loadMesh(4);
                break;

            case GLFW_KEY_6:
                printf("\nLOADING MESH: HOUSE\n");
                loadMesh(5);
                break;

            case GLFW_KEY_7:
                printf("\nLOADING MESH: SHARK\n");
                loadMesh(6);
                break;

            case GLFW_KEY_8:
                printf("\nLOADING MESH: GLASSES\n");
                loadMesh(7);
                break;

            case GLFW_KEY_9:
                printf("\nLOADING MESH: SPOOKY SCARY SKELETON\n");
                loadMesh(8);
                break;

            case GLFW_KEY_0:
                printf("\nLOADING MESH: MONSTER TRUCK\n");
                loadMesh(9);
                break;

            //TRANSLATION
            case GLFW_KEY_UP:
                translateObject(selectedIndex, 0.0, 0.2, 0.0);
                break;

            case GLFW_KEY_DOWN:
                translateObject(selectedIndex, 0.0, -0.2, 0.0);
                break;

            case GLFW_KEY_LEFT:
                translateObject(selectedIndex, -0.2, 0.0, 0.0);
                break;

            case GLFW_KEY_RIGHT:
                translateObject(selectedIndex, 0.2, 0.0, 0.0);
                break;

            case GLFW_KEY_EQUAL:
            case GLFW_KEY_KP_ADD:
                translateObject(selectedIndex, 0.0, 0.0, 0.2);
                break;

            case GLFW_KEY_MINUS:
            case GLFW_KEY_KP_SUBTRACT:
                translateObject(selectedIndex, 0.0, 0.0, -0.2);
                break;

            //ROTATION
            case GLFW_KEY_T:
                rotateObject(selectedIndex, 0.2, 0.0, 0.0);
                break;

            case GLFW_KEY_Y:
                rotateObject(selectedIndex, -0.2, 0.0, 0.0);
                break;
            
            case GLFW_KEY_U:
                rotateObject(selectedIndex, 0.0, 0.2, 0.0);
                break;

            case GLFW_KEY_I:
                rotateObject(selectedIndex, 0.0, -0.2, 0.0);
                break;

            case GLFW_KEY_O:
                rotateObject(selectedIndex, 0.0, 0.0, 0.2);
                break;

            case GLFW_KEY_P:
                rotateObject(selectedIndex, 0.0, 0.0, -0.2);
                break;

            //SCALING
            case GLFW_KEY_L:
                scaleObject(selectedIndex, 1.2);
                break;

            case GLFW_KEY_K:
                scaleObject(selectedIndex, 0.8);
                break;

            //CAMERA CONTROL
            case GLFW_KEY_A:
                printf("Left\n");
                camPosX += 0.2;
                break;

            case GLFW_KEY_D:
                printf("Right\n");
                camPosX -= 0.2;
                break;

            case GLFW_KEY_E:
                printf("Down\n");
                camPosY += 0.2;
                break;

            case GLFW_KEY_Q:
                printf("Up\n");
                camPosY -= 0.2;
                break;

            case GLFW_KEY_W:
                printf("Forward\n");
                camPosZ += 0.2;
                break;
                
            case GLFW_KEY_S:
                printf("Back\n");
                camPosZ -= 0.2;
                break;

            case GLFW_KEY_C:
                printf("Turn Left\n");
                camFX += 0.2;
                break;
                
            case GLFW_KEY_Z:
                printf("Turn Right\n");
                camFX -= 0.2;
                break;

            case GLFW_KEY_F1:
                if(selectedIndex != -1){
                    for(i=0; i < V.cols(); i++){
                        if((V.coeff(1,selectedVertexID)==V.coeff(1,i)) && (V.coeff(2,selectedVertexID)==V.coeff(2,i)) && (V.coeff(3,selectedVertexID)==V.coeff(3,i))){
                            C.col(i) << 1.0, 0.2, 0.2, 0.8;
                            for(j=0; j<selectedVertexIDandCols.cols(); j++){
                                if(selectedVertexIDandCols.coeff(0,j) == i){
                                    selectedVertexIDandCols.col(j) << i, C.coeff(0,i), C.coeff(1,i), C.coeff(2,i), 0.8;
                                }
                            }
                        }
                    }
                }
                break;

            case GLFW_KEY_F2:
                if(selectedIndex != -1){
                    for(i=0; i < V.cols(); i++){
                        if((V.coeff(1,selectedVertexID)==V.coeff(1,i)) && (V.coeff(2,selectedVertexID)==V.coeff(2,i)) && (V.coeff(3,selectedVertexID)==V.coeff(3,i))){
                            C.col(i) << 0.2, 1.0, 0.2, 0.8;
                            for(j=0; j<selectedVertexIDandCols.cols(); j++){
                                if(selectedVertexIDandCols.coeff(0,j) == i){
                                    selectedVertexIDandCols.col(j) << i, C.coeff(0,i), C.coeff(1,i), C.coeff(2,i), 0.8;
                                }
                            }
                        }
                    }
                }
                break;

            case GLFW_KEY_F3:
                if(selectedIndex != -1){
                    for(i=0; i < V.cols(); i++){
                        if((V.coeff(1,selectedVertexID)==V.coeff(1,i)) && (V.coeff(2,selectedVertexID)==V.coeff(2,i)) && (V.coeff(3,selectedVertexID)==V.coeff(3,i))){
                            C.col(i) << 1.0, 0.2, 1.0, 0.8;
                            for(j=0; j<selectedVertexIDandCols.cols(); j++){
                                if(selectedVertexIDandCols.coeff(0,j) == i){
                                    selectedVertexIDandCols.col(j) << i, C.coeff(0,i), C.coeff(1,i), C.coeff(2,i), 0.8;
                                }
                            }
                        }
                    }
                }
                break;

            case GLFW_KEY_F4:
                if(selectedIndex != -1){
                    for(i=0; i < V.cols(); i++){
                        if((V.coeff(1,selectedVertexID)==V.coeff(1,i)) && (V.coeff(2,selectedVertexID)==V.coeff(2,i)) && (V.coeff(3,selectedVertexID)==V.coeff(3,i))){
                            C.col(i) << 1.0, 1.0, 0.2, 0.8;
                            for(j=0; j<selectedVertexIDandCols.cols(); j++){
                                if(selectedVertexIDandCols.coeff(0,j) == i){
                                    selectedVertexIDandCols.col(j) << i, C.coeff(0,i), C.coeff(1,i), C.coeff(2,i), 0.8;
                                }
                            }
                        }
                    }
                }
                break;

            case GLFW_KEY_KP_4:
                if(selectedIndex != -1){
                    xVals = V.coeff(1,selectedVertexID);
                    yVals = V.coeff(2,selectedVertexID);
                    zVals = V.coeff(3,selectedVertexID);
                    for(i=0; i < V.cols(); i++){
                        if((V.coeff(1,i)==xVals) && (V.coeff(2,i)==yVals) && (V.coeff(3,i)==zVals)){
                            V.col(i) << V.coeff(0,i), V.coeff(1,i)-0.05, V.coeff(2,i), V.coeff(3,i);
                        }
                    }
                    for(i=0; i < VLines.cols(); i++){
                        if((VLines.coeff(1,i)==xVals) && (VLines.coeff(2,i)==yVals) && (VLines.coeff(3,i)==zVals)){
                            VLines.col(i) << VLines.coeff(0,i), VLines.coeff(1,i)-0.05, VLines.coeff(2,i), VLines.coeff(3,i);
                        }
                    }
                }
                break;

            case GLFW_KEY_KP_6:
                if(selectedIndex != -1){
                    xVals = V.coeff(1,selectedVertexID);
                    yVals = V.coeff(2,selectedVertexID);
                    zVals = V.coeff(3,selectedVertexID);
                    for(i=0; i < V.cols(); i++){
                        if((V.coeff(1,i)==xVals) && (V.coeff(2,i)==yVals) && (V.coeff(3,i)==zVals)){
                            V.col(i) << V.coeff(0,i), V.coeff(1,i)+0.05, V.coeff(2,i), V.coeff(3,i);
                        }
                    }
                    for(i=0; i < VLines.cols(); i++){
                        if((VLines.coeff(1,i)==xVals) && (VLines.coeff(2,i)==yVals) && (VLines.coeff(3,i)==zVals)){
                            VLines.col(i) << VLines.coeff(0,i), VLines.coeff(1,i)+0.05, VLines.coeff(2,i), VLines.coeff(3,i);
                        }
                    }
                }
                break;

            case GLFW_KEY_KP_8:
                if(selectedIndex != -1){
                    xVals = V.coeff(1,selectedVertexID);
                    yVals = V.coeff(2,selectedVertexID);
                    zVals = V.coeff(3,selectedVertexID);
                    for(i=0; i < V.cols(); i++){
                        if((V.coeff(1,i)==xVals) && (V.coeff(2,i)==yVals) && (V.coeff(3,i)==zVals)){
                            V.col(i) << V.coeff(0,i), V.coeff(1,i), V.coeff(2,i)+0.05, V.coeff(3,i);
                        }
                    }
                    for(i=0; i < VLines.cols(); i++){
                        if((VLines.coeff(1,i)==xVals) && (VLines.coeff(2,i)==yVals) && (VLines.coeff(3,i)==zVals)){
                            VLines.col(i) << VLines.coeff(0,i), VLines.coeff(1,i), VLines.coeff(2,i)+0.05, VLines.coeff(3,i);
                        }
                    }
                }
                break;

            case GLFW_KEY_KP_2:
                if(selectedIndex != -1){
                    xVals = V.coeff(1,selectedVertexID);
                    yVals = V.coeff(2,selectedVertexID);
                    zVals = V.coeff(3,selectedVertexID);
                    for(i=0; i < V.cols(); i++){
                        if((V.coeff(1,i)==xVals) && (V.coeff(2,i)==yVals) && (V.coeff(3,i)==zVals)){
                            V.col(i) << V.coeff(0,i), V.coeff(1,i), V.coeff(2,i)-0.05, V.coeff(3,i);
                        }
                    }
                    for(i=0; i < VLines.cols(); i++){
                        if((VLines.coeff(1,i)==xVals) && (VLines.coeff(2,i)==yVals) && (VLines.coeff(3,i)==zVals)){
                            VLines.col(i) << VLines.coeff(0,i), VLines.coeff(1,i), VLines.coeff(2,i)-0.05, VLines.coeff(3,i);
                        }
                    }
                }
                break;

            case GLFW_KEY_KP_7:
                if(selectedIndex != -1){
                    xVals = V.coeff(1,selectedVertexID);
                    yVals = V.coeff(2,selectedVertexID);
                    zVals = V.coeff(3,selectedVertexID);
                    for(i=0; i < V.cols(); i++){
                        if((V.coeff(1,i)==xVals) && (V.coeff(2,i)==yVals) && (V.coeff(3,i)==zVals)){
                            V.col(i) << V.coeff(0,i), V.coeff(1,i), V.coeff(2,i), V.coeff(3,i)-0.05;
                        }
                    }
                    for(i=0; i < VLines.cols(); i++){
                        if((VLines.coeff(1,i)==xVals) && (VLines.coeff(2,i)==yVals) && (VLines.coeff(3,i)==zVals)){
                            VLines.col(i) << VLines.coeff(0,i), VLines.coeff(1,i), VLines.coeff(2,i), VLines.coeff(3,i)-0.05;
                        }
                    }
                }
                break;

            case GLFW_KEY_KP_9:
                if(selectedIndex != -1){
                    xVals = V.coeff(1,selectedVertexID);
                    yVals = V.coeff(2,selectedVertexID);
                    zVals = V.coeff(3,selectedVertexID);
                    for(i=0; i < V.cols(); i++){
                        if((V.coeff(1,i)==xVals) && (V.coeff(2,i)==yVals) && (V.coeff(3,i)==zVals)){
                            V.col(i) << V.coeff(0,i), V.coeff(1,i), V.coeff(2,i), V.coeff(3,i)+0.05;
                        }
                    }
                    for(i=0; i < VLines.cols(); i++){
                        if((VLines.coeff(1,i)==xVals) && (VLines.coeff(2,i)==yVals) && (VLines.coeff(3,i)==zVals)){
                            VLines.col(i) << VLines.coeff(0,i), VLines.coeff(1,i), VLines.coeff(2,i), VLines.coeff(3,i)+0.05;
                        }
                    }
                }
                break;

            case GLFW_KEY_X:
                if(selectedIndex != -1){
                    printf("DELETING VERTEX VIA EDGE COLLAPSE\n");
                    xVals = V.coeff(1,selectedVertexID);
                    yVals = V.coeff(2,selectedVertexID);
                    zVals = V.coeff(3,selectedVertexID);
                    newXVals = V.coeff(1,selectedVertexID);
                    newYVals = V.coeff(2,selectedVertexID);
                    newZVals = V.coeff(3,selectedVertexID);

                    distance = -1;
                    minDistance = -1;
                    for(i=0; i<V.cols(); i+=3){
                        if(vertexInTriangle(selectedVertexID, i)){
                            printf("TRIANGLE FOUND\n");
                            distance = getSquaredDistance(V.coeff(1,i), V.coeff(2,i), V.coeff(3,i), V.coeff(1,selectedVertexID), V.coeff(2,selectedVertexID), V.coeff(3,selectedVertexID));
                            if((minDistance == -1 || distance < minDistance) && distance != 0.0){
                                minDistance = distance;
                                newXVals = V.coeff(1,i);
                                newYVals = V.coeff(2,i);
                                newZVals = V.coeff(3,i);
                            }
                        }
                    }

                    printf("COLLAPSING EDGE: %f, %f, %f\t%f, %f, %f\n",xVals, yVals, zVals, newXVals, newYVals, newZVals);

                    for(i=0; i < V.cols(); i++){
                        if((V.coeff(1,i)==xVals) && (V.coeff(2,i)==yVals) && (V.coeff(3,i)==zVals)){
                            V.col(i) << V.coeff(0,i), newXVals, newYVals, newZVals;
                        }
                    }

                    for(i=0; i < VLines.cols(); i++){
                        if((VLines.coeff(1,i)==xVals) && (VLines.coeff(2,i)==yVals) && (VLines.coeff(3,i)==zVals)){
                            VLines.col(i) << VLines.coeff(0,i), newXVals, newYVals, newZVals;
                        }
                    }
                }
                break;

            case GLFW_KEY_SPACE:
                dynamicLight = !dynamicLight;
                break;

            default:
                break;
        }
    }
    VBO.update(V);
}

int main(void){
    GLFWwindow* window;

    // Initialize the library
    if (!glfwInit())
        return -1;

    // Activate supersampling
    glfwWindowHint(GLFW_SAMPLES, 8);

    // Ensure that we get at least a 3.2 context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

    // On apple we have to load a core profile with forward compatibility

    #ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif

    // Create a windowed mode window and its OpenGL context
    window = glfwCreateWindow(640, 480, "Final Project Richanshu Jha", NULL, NULL);
    if (!window){
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    #ifndef __APPLE__
      glewExperimental = true;
      GLenum err = glewInit();
      if(GLEW_OK != err){
        /* Problem: glewInit failed, something is seriously wrong. */
       fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
      }
      glGetError(); // pull and savely ignonre unhandled errors like GL_INVALID_ENUM
      fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
    #endif

    int major, minor, rev;
    major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
    minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
    rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
    printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
    printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
    printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

    VertexArrayObject VAO;
    VAO.init();
    VAO.bind();

    VBO.init();
    VBOColor.init();
    VBOVFlatNormals.init();

    VBO.update(V);
    VBOColor.update(C);
    VBOVFlatNormals.update(VFlatNormals);

    Program program;

    const GLchar* vertex_shader =
            "#version 150 core\n"
                    "in vec4 position;"
                    "in vec4 color;"
                    "in vec3 FlatNormals;"
                    "uniform vec4 camRight;"
                    "uniform vec4 camUp;"
                    "uniform vec4 camForward;"
                    "uniform vec4 camPos;"
                    "uniform vec3 oscillator;"
                    "out vec4 colorOut;"

                    //WRITING MATRIX TRANSPOSES
                    "mat4 positionMatrix = mat4("
                        "position[1], position[2], position[3], 1.0,"
                        "FlatNormals[0], FlatNormals[1], FlatNormals[2], 0.0,"
                        "0.0, 0.0, 1.0, 0.0,"
                        "0.0, 0.0, 0.0, 1.0"
                    ");"
                    
                    // CAMERA POSITION
                    "mat4 viewMatrix = mat4("
                    "   camRight[0], camRight[1], camRight[2], camRight[3],"
                    "   camUp[0], camUp[1], camUp[2], camUp[3],"
                    "   camForward[0], camForward[1], camForward[2], camForward[3],"
                    "   camPos[0], camPos[1], camPos[2], camPos[3]" 
                    ");"

                    "mat4 resultMatrix = mat4(viewMatrix * positionMatrix);"
                    
                    "vec3 lightSource = vec3(0.0+oscillator[0], 1.3+oscillator[1], 0.0);"
                    "float amb = 0.3;"
                    "vec4 colorShaded = vec4(0.0, 0.0, 0.0, 0.0);"
                    
                    "void main()"
                    "{"
                    "   if(color[3] > 0.9){" //LINE AND SELECTION HIGHLIGHT (THAT DONT NEED LIGHTING)
                    "       colorShaded = vec4(color[0], color[1], color[2], 1.0);"
                    "   }"
                    "   else if(color[3] > 0.75){" //FLAT SHADING
                    "       vec3 iVec = vec3(normalize(vec3(resultMatrix[0][0], resultMatrix[0][1], resultMatrix[0][2]) - lightSource));"
                    "       vec3 nVec = vec3(normalize(vec3(resultMatrix[1][0], resultMatrix[1][1], resultMatrix[1][2])));"
                    "       float x = dot(iVec,nVec);"
                    "       colorShaded = vec4(amb + (color[0] * x) , amb + (color[1] * x), amb + (color[2] * x), 1.0);"
                    "   }"
                    "   colorOut = colorShaded;"
                    "   gl_Position = vec4(resultMatrix[0][0], resultMatrix[0][1], resultMatrix[0][2], 1.0);"
                    "}";

    const GLchar* fragment_shader =
            "#version 150 core\n"
                    "in vec4 colorOut;"
                    "out vec4 outColor;"
                    "void main()"
                    "{"
                    "    if(colorOut.a < 0.1){"
                    "       discard;"
                    "    }"
                    "    outColor = colorOut;"
                    "}";

    program.init(vertex_shader,fragment_shader,"outColor");
    program.bind();
    program.bindVertexAttribArray("position",VBO);
    program.bindVertexAttribArray("color",VBOColor);
    program.bindVertexAttribArray("FlatNormals",VBOVFlatNormals);

    auto t_start = std::chrono::high_resolution_clock::now();

    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

    // Register the mouse callback
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    // Update viewport
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(1);

    // Loop until the user closes the window
    while (!glfwWindowShouldClose(window)){
        // Set the uniform value depending on the time difference
        auto t_now = std::chrono::high_resolution_clock::now();
        timeGlobal = std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count();
        
        if(dynamicLight){glUniform3f(program.uniform("oscillator"), 2*(float)(sin(timeGlobal * 2.0f) + 1.0f), 2*(float)(cos(timeGlobal * 2.0f) + 1.0f), 0.0f);}
        else{glUniform3f(program.uniform("oscillator"), 0.0f, 0.0f, 0.0f);}

        glUniform4f(program.uniform("camRight"), 1.0, 0.0, 0.0, 0.0);
        glUniform4f(program.uniform("camUp"), 0.0, 1.0, 0.0, 0.0);
        glUniform4f(program.uniform("camForward"), camFX, camFY, camFZ, 0.0);
        glUniform4f(program.uniform("camPos"), camPosX, camPosY, camPosZ, 1.0);

        // Bind your VAO (not necessary if you have only one)
        VAO.bind();

        glEnable(GL_DEPTH_TEST);

        // Bind your program
        program.bind();

        // Clear the framebuffer
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        //Get cursor position
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        cursorX = (((xpos/double(width))*2)-1)-camPosX;
        cursorY = ((((height-1-ypos)/double(height))*2)-1)-camPosY; // NOTE: y axis is flipped in glfw

        // Draw triangles
        VBO.update(V);
        VBOColor.update(C);
        VBOVFlatNormals.update(VFlatNormals);
        glDrawArrays(GL_TRIANGLES, 0, V.cols());   
        
        glLineWidth(1.0);
        // Draw Lines
        VBO.update(VLines);
        VBOColor.update(CLines);
        VBOVFlatNormals.update(LineCentroids); //Filling with values. Wont be used
        glDrawArrays(GL_LINES, 0, VLines.cols());
        
        if(selectedIndex != -1){
            // Draw Cursor Lines
            VCursorLine.col(0) << 0.0, cursorX, cursorY, cursorZ;
            VCursorLine.col(1) << 1.0, V.coeff(1,selectedVertexID), V.coeff(2,selectedVertexID), V.coeff(3,selectedVertexID);

            CCursorLine.col(0) << 1.0, 0.3, 0.2, 1.0;
            CCursorLine.col(1) << 1.0, 0.3, 0.2, 1.0;

            CursorTemp.col(0) << 0.0, 0.0, 0.0;
            CursorTemp.col(1) << 0.0, 0.0, 0.0;

            glLineWidth(1.3);
            VBO.update(VCursorLine);
            VBOColor.update(CCursorLine);
            VBOVFlatNormals.update(CursorTemp); //Filling with values. Wont be used
            glDrawArrays(GL_LINES, 0, VCursorLine.cols());
        }

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();

        //CLEARING DEPTH BUFFER
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    // Deallocate opengl memory
    program.free();
    VAO.free();
    VBO.free();
    VBOColor.free();
    VBOVFlatNormals.free();

    // Deallocate glfw internals
    glfwTerminate();
    return 0;
}