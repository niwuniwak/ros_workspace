/*
 * mycloud.h
 *
 *  Created on: April, 2012
 *      Author: waqar shahid
 */
 
#ifndef MYCLOUD_H_
#define MYCLOUD_H_

 #include <stdio.h>
 #include<iostream>
 #include <vector>
 #include<string>

using namespace std;

typedef struct { int regionID; float x; float y; float z;} point_3D;
typedef struct{ int CID1; float x; float y;} point_2D;
typedef struct{ point_3D my3D; int num_2D; vector<point_2D> my2D; int inlier;} SceneData;
typedef struct {int frameId; double rotation[4];double translation[3];} camera;

class mycloud{

    public:
    vector<SceneData> point_data;
    vector<camera> cameras;
    mycloud();
     ~mycloud();
    mycloud& operator=(const mycloud &other);
    friend std::ostream& operator<<(std::ostream& strm, const mycloud& a);
    friend std::istream& operator>>(std::istream& strm, mycloud& a);

};
#endif /* MYCLOUD_H_ */



