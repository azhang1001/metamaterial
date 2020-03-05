#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
using namespace std;

vector<float> multiply_quaternion(vector<float> q, vector<float> r);
vector<float> inverse_quaternion(vector<float> q);
vector<vector<float>> rotate_points(vector<vector<float>> vertices, float theta);
void generate_inflation(ofstream& outputFile, ifstream& inputFile, float percent);