using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

	int[] 		Tet;
	int tet_number;			//The number of tetrahedra

	float[] inv_dm_det;

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	float restitution = 0.5f;    // for collision
	float mu_t =0.5f;

	SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
		
		/*
        tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);
		*/

        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear();
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

		//TODO: Need to allocate and assign inv_Dm
		inv_Dm = new Matrix4x4[tet_number];
		for(int tet = 0; tet < tet_number; ++tet) {
			inv_Dm[tet] = Build_Edge_Matrix(tet);
		}

		inv_dm_det = new float[tet_number];
		for(int tet = 0; tet < tet_number; ++tet) {
			int i, j, k, l;
			i = Tet[tet * 4 + 0];
			j = Tet[tet * 4 + 1];
			k = Tet[tet * 4 + 2];
			l = Tet[tet * 4 + 3];
			inv_dm_det[tet] = Mathf.Abs(Vector3.Dot(Vector3.Cross(X[j] - X[i], X[k] - X[i]), X[l] - X[i]));
		}
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
    	//TODO: Need to build edge matrix here.
		int i, j, k, l;
		i = Tet[tet * 4 + 0];
		j = Tet[tet * 4 + 1];
		k = Tet[tet * 4 + 2];
		l = Tet[tet * 4 + 3];

		if(tet == 0) {
			Debug.Log("7777777777777777777 " + (X[j] - X[i]));
			Debug.Log("8888888888888888888 " + (X[k] - X[i]));
			Debug.Log("9999999999999999999 " + (X[l] - X[i]));
		}

		ret.SetColumn(0, X[j] - X[i]);
		ret.SetColumn(1, X[k] - X[i]);
		ret.SetColumn(2, X[l] - X[i]);
		ret[3, 3] = 1.0f;

		return ret.inverse;
    }

	Matrix4x4 Matrix_Subtract_Matrix(Matrix4x4 lhs, Matrix4x4 rhs) {
		Matrix4x4 ret = Matrix4x4.identity;
		ret[0, 0] = lhs[0, 0] - rhs[0, 0];
		ret[0, 1] = lhs[0, 1] - rhs[0, 1];
		ret[0, 2] = lhs[0, 2] - rhs[0, 2];
		ret[1, 0] = lhs[1, 0] - rhs[1, 0];
		ret[1, 1] = lhs[1, 1] - rhs[1, 1];
		ret[1, 2] = lhs[1, 2] - rhs[1, 2];
		ret[2, 0] = lhs[2, 0] - rhs[2, 0];
		ret[2, 1] = lhs[2, 1] - rhs[2, 1];
		ret[2, 2] = lhs[2, 2] - rhs[2, 2];
		return ret;
	}

	Matrix4x4 Matrix_Add_Matrix(Matrix4x4 lhs, Matrix4x4 rhs) {
		Matrix4x4 ret = Matrix4x4.identity;
		ret[0, 0] = lhs[0, 0] + rhs[0, 0];
		ret[0, 1] = lhs[0, 1] + rhs[0, 1];
		ret[0, 2] = lhs[0, 2] + rhs[0, 2];
		ret[1, 0] = lhs[1, 0] + rhs[1, 0];
		ret[1, 1] = lhs[1, 1] + rhs[1, 1];
		ret[1, 2] = lhs[1, 2] + rhs[1, 2];
		ret[2, 0] = lhs[2, 0] + rhs[2, 0];
		ret[2, 1] = lhs[2, 1] + rhs[2, 1];
		ret[2, 2] = lhs[2, 2] + rhs[2, 2];
		return ret;
	}

	Matrix4x4 Matrix_Multiply_Scalar(Matrix4x4 m, float scalar)
	{
		Matrix4x4 A = m;
		A[0, 0] *= scalar;
		A[0, 1] *= scalar;
		A[0, 2] *= scalar;
		A[1, 0] *= scalar;
		A[1, 1] *= scalar;
		A[1, 2] *= scalar;
		A[2, 0] *= scalar;
		A[2, 1] *= scalar;
		A[2, 2] *= scalar;

		return A;
	}

	float Matrix_Trace(Matrix4x4 m)
	{
		return m[0, 0] + m[1, 1] + m[2, 2];
	}

    void _Update()
    {	
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
    		//TODO: Add gravity to Force.
			Force[i] = mass * new Vector3(0.0f, -9.8f, 0.0f);
			V[i] *= damp;
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
    		//TODO: Deformation Gradient
			int i, j, k, l;
			i = Tet[tet * 4 + 0];
			j = Tet[tet * 4 + 1];
			k = Tet[tet * 4 + 2];
			l = Tet[tet * 4 + 3];
			Vector3 xij = X[j] - X[i];
			Vector3 xik = X[k] - X[i];
			Vector3 xil = X[l] - X[i];
    		Matrix4x4 deformMatrix = Matrix4x4.zero;
			deformMatrix[0, 0] = xij.x;
			deformMatrix[1, 0] = xij.y;
			deformMatrix[2, 0] = xij.z;
			deformMatrix.SetColumn(0, xij);
			deformMatrix.SetColumn(1, xik);
			deformMatrix.SetColumn(2, xil);
			deformMatrix[3, 3] = 1.0f;

			Matrix4x4 F = deformMatrix * inv_Dm[tet];
			if(tet == 0) {
				Debug.Log("111111111111111111 deformMatrix\n " + deformMatrix);
				Debug.Log("222222222222222222 F\n " + F);
				Debug.Log("333333333333333333 inv_Dm\n" + inv_Dm[tet]);
			}
			// F = Matrix4x4.identity;
    		//TODO: Green Strain
			Matrix4x4 U = Matrix4x4.identity;
			Matrix4x4 D = Matrix4x4.identity;
			Matrix4x4 V = Matrix4x4.identity;
			svd.svd(F, ref U, ref D, ref V);

			Matrix4x4 G = Matrix_Multiply_Scalar(Matrix_Subtract_Matrix(F.transpose * F, Matrix4x4.identity), 0.5f);
			// Matrix4x4 G = Matrix_Multiply_Scalar(Matrix_Subtract_Matrix(U * D * D * V.transpose, Matrix4x4.identity), 0.5f);
			if(tet == 0) {
				Debug.Log("777777777777777777 " + G + " " + Matrix_Trace(G));
			}
    		//TODO: Second PK Stress
			Matrix4x4 S = Matrix_Add_Matrix(Matrix_Multiply_Scalar(G, 2.0f * stiffness_1), Matrix_Multiply_Scalar(Matrix4x4.identity, stiffness_0 * Matrix_Trace(G)));
			if(tet == 0) {
				Debug.Log("aaaaaaaaaaaaaaaaaaa " + S);
				Debug.Log("bbbbbbbbbbbbbbbbbbb " + Matrix_Multiply_Scalar(G, 2.0f * stiffness_1));
				Debug.Log("ccccccccccccccccccc " + Matrix_Multiply_Scalar(Matrix4x4.identity, stiffness_0 * Matrix_Trace(G)));
			}
    		//TODO: Elastic Force
			Matrix4x4 f = Matrix_Multiply_Scalar(F * S * inv_Dm[tet].transpose, -inv_dm_det[tet] / 6.0f);
			Vector3 f1, f2, f3;
			f1 = new Vector3(f[0, 0], f[1, 0], f[2, 0]);
			f2 = new Vector3(f[0, 1], f[1, 1], f[2, 1]);
			f3 = new Vector3(f[0, 2], f[1, 2], f[2, 2]);
			if(tet == 0) {
				Debug.Log("9999999999999999999 " + f1 + " " + f2 + " " + f3);
			}
			
			Force[j] += f1;
			Force[k] += f2;
			Force[l] += f3;
			Force[i] += -f1 - f2 - f3;
    	}

    	for(int i=0; i < number; i++)
    	{
			// Debug.Log("hhhhhhhhhhhhhhhhhhhhh " + i + " " + Force[i]);
    		//TODO: Update X and V here.
			V[i] += Force[i] / mass * dt;
    		//TODO: (Particle) collision with floor.
    	}

		// laplacian smoothing
		for(int i = 0; i < number; ++i) {
			V_num[i] = 0;
			V_sum[i] = new Vector3(0.0f, 0.0f, 0.0f);
		}
		for(int tet = 0; tet < tet_number; ++tet) {
			int i, j, k, l;
			i = Tet[tet * 4 + 0];
			j = Tet[tet * 4 + 1];
			k = Tet[tet * 4 + 2];
			l = Tet[tet * 4 + 3];

			Vector3 total_velocity = V[i] + V[j] + V[k] + V[l];

			V_num[i] += 3;
			V_sum[i] += V[j] + V[k] + V[l];
			V_num[j] += 3;
			V_sum[j] += V[i] + V[k] + V[l];
			V_num[k] += 3;
			V_sum[k] += V[i] + V[j] + V[l];
			V_num[l] += 3;
			V_sum[l] += V[i] + V[j] + V[k];
		}

		for(int i = 0; i < number ; ++i) {
			V[i] = V[i] * 0.9f + V_sum[i] / V_num[i] * 0.1f;
		}
		
		for(int i=0; i < number; i++)
    	{
    		//TODO: Update X and V here.
			X[i] += V[i] * dt;
    		//TODO: (Particle) collision with floor.
    	}

		Collision_Plane(new Vector3(0, -3.01f, 0), new Vector3(0, 1, 0));
    }

	void Collision_Plane(Vector3 P, Vector3 N)
	{
		Vector3 p_normal, p_tangent;
		Vector3 v_normal, v_tangent, v_normal_new, v_tangent_new;
		for(int i = 0; i < X.Length; ++i) {
			if(Vector3.Dot(X[i] - P, N) > 0.0f) {
				continue;
			}
			if(Vector3.Dot(V[i], N) > 0.0f) {
				continue;
			}
			v_normal = Vector3.Dot(V[i], N) * N;
			v_tangent = V[i] - v_normal;
			float a = Mathf.Max(1.0f - mu_t * (1.0f + restitution) * v_normal.magnitude / v_tangent.magnitude, 0.0f);
			v_normal_new = -restitution * v_normal;
			v_tangent_new = a * v_tangent;
			V[i] = v_normal_new + v_tangent_new;

			p_normal = Vector3.Dot(X[i] - P, N) * N;
			p_tangent = X[i] - P - p_normal;
			X[i] = P + p_tangent + (-restitution) * p_normal;
		}
	}

    // Update is called once per frame
    void Update()
    {
		// dt = Time.deltaTime / 10.0f;
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }
}
