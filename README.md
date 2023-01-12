# GAMES103
The assignments of GAMES103 (Introduction to Physically based Animation) whose instructor is Huamin Wang, if you are interested, please refer to http://games-cn.org/games103/ for more information. this lecture mainly includes six parts: rigid body dynamics and contacts, mass-spring systems, constrained approaches, collision handling, soft body dynamics(finite element method) and fluid dynamics.

## Angry Bunny
In this assignment, I have implemented rigid body dynamics, and to solve the contacts between the bunny and the surrounding environment, I use two methods: impulse method and shape matching method.

### Impulse Method
Here is the final result of impluse method:
![impulse_method](lab1/results/impulse_method.gif)


### Shape Matching Method
Here is the final result of shape matching method:
![shape_matching](lab1/results/shape_matching.gif)
A difference between this method and the first one is that, to tranform the bunny, the first method is modifying the bunny's position and orientation, however, In this method, the bunny's poisition and orientation are set to identity, it modifies bunny vertices posiiton for every frame, which maybe a little clumsy and time consuming. another side effect that is prone to ignore is that the shading is wrong caused by not updating the normal of all vertices. so you may add several lines of code in the Update_Mesh method to solve this problem. note how the normals are updated. (causion: do not apply the translation to the normal)

```C#
void Update_Mesh(Vector3 c, Matrix4x4 R, float inv_dt)
   	{
   		for(int i=0; i<Q.Length; i++)
		{
			Vector3 x=(Vector3)(R*Q[i])+c;

			V[i]+=(x-X[i])*inv_dt;
			if(i == 0) {
				Debug.Log("UpdateMesh " + V[i]);
			}
			X[i]=x;
		}
		for(int i = 0; i < N.Length; i++) {
			current_normals[i] = R * N[i];
		}
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices=X;
		mesh.normals = current_normals;
   	}
```

there is a common problem in these two implementations, which is caused by delta time provided by the based code, the delta time in the base code is a fixed number 0.016 seconds standing for the time executed every frame. If you have a fast CPU, it may take only 3 or 4 millliseconds to exectue a frame, if this is the case, the bunny will have a higher velocity than you want. as a result, using the actual delta time will solve this problem perfectly.
In Unity, you can use this function to get the delta time in the Update function.

```C#
dt = Time.deltaTime;
```