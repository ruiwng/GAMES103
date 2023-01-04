using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision

	float mu_t = 0.5f;
	// Use this for initialization
	bool stable_state = false;
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
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

	Matrix4x4 Matrix_Subtract_Matrix(Matrix4x4 m, Matrix4x4 n)
	{
		Matrix4x4 A = Matrix4x4.identity;
		A[0, 0] = m[0, 0] - n[0, 0];
		A[0, 1] = m[0, 1] - n[0, 1];
		A[0, 2] = m[0, 2] - n[0, 2];
		A[1, 0] = m[1, 0] - n[1, 0];
		A[1, 1] = m[1, 1] - n[1, 1];
		A[1, 2] = m[1, 2] - n[1, 2];
		A[2, 0] = m[2, 0] - n[2, 0];
		A[2, 1] = m[2, 1] - n[2, 1];
		A[2, 2] = m[2, 2] - n[2, 2];
		return A;
	}

	Quaternion Quaternion_Add_Quaternion(Quaternion m, Quaternion n)
	{
		return new Quaternion(m.x + n.x, m.y + n.y, m.z + n.z, m.w + n.w);
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;
		int collision_count = 0;
		Vector3 final_v = new Vector3(0.0f, 0.0f, 0.0f);
		Vector3 final_w = new Vector3(0.0f, 0.0f, 0.0f);
		Vector3 pos = new Vector3(0.0f, 0.0f, 0.0f);

		Matrix4x4 rotation = Matrix4x4.Rotate(transform.rotation);
		Vector3 position = transform.position;
		Vector3 v_direction, v_position, v_velocity;

		for(int i = 0; i < vertices.Length; i++) {
			v_direction = rotation.MultiplyPoint(vertices[i]);
			v_position = position + v_direction;
			if(Vector3.Dot(v_position - P, N) > 0.0f) {
				continue;
			}
			v_velocity = v + Vector3.Cross(w, v_direction);
			if(Vector3.Dot(v_velocity, N) > 0.0f) {
				continue;
			}
			collision_count += 1;
			pos += vertices[i];
		}

		if(collision_count == 0) {
			return;
		}
		
		Vector3 v_local_position = pos / collision_count;
		v_direction = rotation.MultiplyPoint(v_local_position);
		v_position = position + v_direction;
		v_velocity = v + Vector3.Cross(w, v_direction);

		Vector3 v_normal = Vector3.Dot(v_velocity, N) * N;
		Vector3 v_tangent = v_velocity - v_normal;
		Debug.Log("velocity " + v + "\n");
		if(v.magnitude < 0.1f && Mathf.Abs(Vector3.Normalize(v).y) > 0.95f) {
			stable_state = true;
		}
		float a = Mathf.Max(1.0f - mu_t * (1.0f + restitution) * v_normal.magnitude / v_tangent.magnitude, 0.0f);
		Vector3 v_normal_new = -restitution * v_normal;
		Vector3 v_tangent_new = a * v_tangent;
		Vector3 v_velocity_new = v_normal_new + v_tangent_new;

		Matrix4x4 cross_matrix = Get_Cross_Matrix(v_direction);
		Matrix4x4 K = Matrix_Subtract_Matrix(Matrix_Multiply_Scalar(Matrix4x4.identity, 1.0f / mass), cross_matrix * I_ref.inverse * cross_matrix);
		Vector3 impulse = K.inverse.MultiplyVector(v_velocity_new - v_velocity);
		// Debug.Log("impluse " + impulse + "\n");
		Debug.Log("restitution " + restitution +" impulse " + impulse + "\n");
		v += impulse * (1.0f / mass);
		w += I_ref.inverse.MultiplyVector(Vector3.Cross(v_direction, impulse));
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r") && launched)
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l") && !launched)
		{
			v = new Vector3 (4.5, 2, 0);
			w = new Vector3(0, 0, 0);
			launched=true;
			stable_state = false;
		}

		if(!launched) {
			return;
		}
		if(stable_state) {
			return;
		}
		// Part I: Update velocities
		v += new Vector3(0.0f, -9.8f, 0.0f) * dt;
		v *= 1.0f - (1.0f - linear_decay) * dt;
		w *= 1.0f - (1.0f - angular_decay) * dt;

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));
		

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x = transform.position;
		x += v * dt;

		//Update angular status
		/*
		Vector3 rotation = w * dt;
		float magnitude = rotation.magnitude;
		float sin_magnitude = Mathf.Sin(magnitude * 0.5f);
		float cos_magnitude = Mathf.Cos(magnitude * 0.5f);
		Vector3 temp = rotation / magnitude * sin_magnitude;
		Quaternion r = new Quaternion(temp.x, temp.y, temp.z, cos_magnitude);
		*/
		Quaternion q = transform.rotation;
		Vector3 angular = w * dt * 0.5f;
		q = Quaternion_Add_Quaternion(q, new Quaternion(angular.x, angular.y, angular.z, 0.0f) * q);
		q = Quaternion.Normalize(q);
		// q = q * r;
		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}
}
