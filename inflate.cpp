#include "inflate.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <vector>

using namespace std;

vector<float> multiply_quaternion(vector<float> q, vector<float> r)
{
	float t0 = r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3];
	float t1 = r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2];
	float t2 = r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1];
	float t3 = r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0];
	return { t0,t1,t2,t3 };
}
vector<float> inverse_quaternion(vector<float> q)
{
	float d = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
	float t0 = q[0] / d;
	float t1 = -q[1] / d;
	float t2 = -q[2] / d;
	float t3 = -q[3] / d;
	return { t0, t1, t2, t3 };
}

vector<vector<float>> rotate_points(vector<vector<float>> vertices, float theta)
{
	// find normal vector
	vector<float> V = { vertices[1][0] - vertices[0][0], vertices[1][1] - vertices[0][1], vertices[1][2] - vertices[0][2] };
	vector<float> W = { vertices[2][0] - vertices[0][0], vertices[2][1] - vertices[0][1], vertices[2][2] - vertices[0][2] };
	//cout << "v is" << V[0] << " " << V[1] << " " << V[2] << "\n";
	//cout << "w is" << W[0] << " " << W[1] << " " << W[2] << "\n";
	// cross product
	vector<float> N = { V[1] * W[2] - V[2] * W[1],V[2] * W[0] - V[0] * W[2] ,V[0] * W[1] - V[1] * W[0] };
	// make unit vector
	float length = sqrt(N[0] * N[0] + N[1] * N[1] + N[2] * N[2]);
	N = { N[0] / length, N[1] / length, N[2] / length };
	//cout << "\n the vector N is" << N[0] << " " << N[1] << " " << N[2] << "\n";
	//N = { 0,0,0.5 };
	vector<float> Q2 = { cosf(theta / 2.0), N[0] * sinf(theta / 2.0), N[1] * sinf(theta / 2.0), N[2] * sinf(theta / 2.0) };
	//cout << cosf(theta);
	vector<vector<float>> new_vertices;
	for (auto& vertex : vertices)
	{
		vector<float> Q1 = { 0,vertex[0],vertex[1],vertex[2] };
		vector<float> Q3 = multiply_quaternion(multiply_quaternion(Q2, Q1), inverse_quaternion(Q2));
		new_vertices.push_back({ Q3[1], Q3[2], Q3[3] });
	}
	return new_vertices;
}

void generate_inflation(ofstream& outputFile, ifstream& inputFile, float percent)
{
	vector<vector<float>> vertices;
	vector<vector<int>> faces;
	vector<vector<float>> new_vertices;
	vector<vector<int>> new_faces;
	string a;
	float rads = percent * 3.14159265f / 3.0f; // must be pi/3 radians = 60 degrees
	float b, c, d;
	vertices.push_back({ -1000,-1000,-1000 }); // so the vertices index starts at 1 instead of 0
	new_vertices.push_back({ -1000,-1000,-1000 }); // so the vertices index starts at 1 instead of 0
	while (inputFile >> a >> b >> c >> d)
	{
		if (a == "f")
		{
			faces.push_back({ int(b), int(c), int(d) });
			break;
		}
		vertices.push_back({ b,c,d });
	}
	int e, f, g;
	while (inputFile >> a >> e >> f >> g)
	{
		faces.push_back({ e,f,g });
	}
	//cout << "done with step 1";
	inputFile.close();
	bool clockwise = false;
	for (auto& face : faces)
	{
		//get each vertex of the triangle
		vector<float> vertex1 = vertices[face[0]];
		vector<float> vertex2 = vertices[face[1]];
		vector<float> vertex3 = vertices[face[2]];

		//find centroid of triangle:
		float centroid_x = (vertex1[0] + vertex2[0] + vertex3[0]) / 3.0f;
		float centroid_y = (vertex1[1] + vertex2[1] + vertex3[1]) / 3.0f;
		float centroid_z = (vertex1[2] + vertex2[2] + vertex3[2]) / 3.0f;


		//move triangle to center(centroid at origin)
		vertex1[0] -= centroid_x;
		vertex1[1] -= centroid_y;
		vertex1[2] -= centroid_z;

		vertex2[0] -= centroid_x;
		vertex2[1] -= centroid_y;
		vertex2[2] -= centroid_z;

		vertex3[0] -= centroid_x;
		vertex3[1] -= centroid_y;
		vertex3[2] -= centroid_z;

		//rotate each triangle by multiplying each coord by -1
		//if (percent == 1.5) 
		//{
		//	vertex1[0] *= -1;
		//	vertex1[1] *= -1;
		//	vertex1[2] *= -1;

		//	vertex2[0] *= -1;
		//	vertex2[1] *= -1;
		//	vertex2[2] *= -1;

		//	vertex3[0] *= -1;
		//	vertex3[1] *= -1;
		//	vertex3[2] *= -1;
		//}

		clockwise = false;
		vector<vector<float>> rotated_vertices = rotate_points({ vertex1,vertex2,vertex3 }, rads);
		vertex1 = rotated_vertices[0];
		vertex2 = rotated_vertices[1];
		vertex3 = rotated_vertices[2];



		//centroid new position: exactly float!
		// other numbers for partial rotations
		//float movement = percent + 1;
		//float movement = sqrt(5 - (percent - 2) * (percent - 2));
		//float movement = sqrt(1 - (percent - 1) * (percent - 1)) + 1;
		float movement = sinf(2.61799 - percent * 1.0472) * 2;
		centroid_x = centroid_x * movement;
		centroid_y = centroid_y * movement;
		centroid_z = centroid_z * movement;

		//move triangle back to correct position
		vertex1[0] += centroid_x;
		vertex1[1] += centroid_y;
		vertex1[2] += centroid_z;

		vertex2[0] += centroid_x;
		vertex2[1] += centroid_y;
		vertex2[2] += centroid_z;

		vertex3[0] += centroid_x;
		vertex3[1] += centroid_y;
		vertex3[2] += centroid_z;

		//check if each vertex is already in new_vertices
		/*int index1 = -1;
		int index2 = -1;
		int index3 = -1;
		if (find(new_vertices.begin(), new_vertices.end(), vertex1) != new_vertices.end())
		{
			for (int i = 0; i < new_vertices.size(); i++)
			{
				if (new_vertices[i] == vertex1)
				{
					index1 = i;
				}
			}
		}
		else
		{
			new_vertices.push_back(vertex1);
			index1 = new_vertices.size() - 1;
		}
		if (find(new_vertices.begin(), new_vertices.end(), vertex2) != new_vertices.end())
		{
			for (int i = 0; i < new_vertices.size(); i++)
			{
				if (new_vertices[i] == vertex2)
				{
					index2 = i;
				}
			}
		}
		else
		{
			new_vertices.push_back(vertex2);
			index2 = new_vertices.size() - 1;
		}
		if (find(new_vertices.begin(), new_vertices.end(), vertex3) != new_vertices.end())
		{
			for (int i = 0; i < new_vertices.size(); i++)
			{
				if (new_vertices[i] == vertex3)
				{
					index3 = i;
				}
			}
		}
		else
		{
			new_vertices.push_back(vertex3);
			index3 = new_vertices.size() - 1;
		}*/
		//infinitely faster program, but many more vertices.
		new_vertices.push_back(vertex1);
		int index1 = new_vertices.size() - 1;
		new_vertices.push_back(vertex2);
		int index2 = new_vertices.size() - 1;
		new_vertices.push_back(vertex3);
		int index3 = new_vertices.size() - 1;
		//make the new faces
		new_faces.push_back({ index1, index2, index3 });
	}
	//cout << "done with step 2";
	new_vertices.erase(new_vertices.begin());
	for (auto& v : new_vertices)
	{
		outputFile << "v ";
		for (auto& x : v)
			outputFile << x << " ";
		outputFile << endl;
	}
	//cout << "done with step 3";
	for (auto& v : new_faces)
	{
		outputFile << "f ";
		for (auto& x : v)
			outputFile << x << " ";
		outputFile << endl;
	}
	//cout << "done with step 4";
	outputFile.close();
}

