# GAMES103

The assignments of GAMES103 (Introduction to Physically based Animation) whose instructor is Huamin Wang, if you are interested, please refer to http://games-cn.org/games103/ for more information. this lecture mainly includes six parts: rigid body dynamics and contacts, mass-spring systems, constrained approaches, collision handling, soft body dynamics(finite element method) and fluid dynamics.

## Angry Bunny

In this assignment, I have implemented rigid body dynamics, and to solve the contacts between the bunny and the surrounding environment, I use two methods: impulse method and shape matching method.

### Impulse Method

Here is the final result of impluse method:
![impulse_method](lab1/results/impulse_method.gif)

Here are the steps to implement this method:

1. For every vertex, we get its position by (where *X* and *R* are the bunny's global position and rotation,  $r_i$ is the local position of very vertex):

$$
X_i = X + Rr_i
$$

2. And then we test every vertex whether the SDF function is greater than 0, if yes, we are done, otherwise go to step 3.

3. For every vertex, we get its velocity by (where $V$ and $w$ are the velocity and angular velocity of the bunny and *R* is the rotation matrix of the bunny):

$$
V_i = V + w \times Rr_i
$$

4. And then we test every vertex's direction whether it moves close to or away from collision object by the following formula, if it's the latter case, we are done, otherwise go to step 5.

$$
v_i * N < 0
$$

5. We compute the new velocity of the vertex, by splitting the vertex's velocity into two parts: one that is parallel to $N$, the other that is perpendicular to $N$, and then handle them respectively.


$$
v_{N, i} = (v_i * N)N
$$

$$
v_{T, i} = v_i - v_{N, i}
$$

$$
a = max(1 - \mu_T(1 + \mu_N)||v_{N,i}||/||v_{T, i}||, 0)
$$

$$
v_{N, i}^{new} = -\mu_Nv_{N, i}
$$

$$
v_{T, i}^{new} = av_{T, i}
$$

$$
v_i^{new} = v_{N, i}^{new} + v_{T, i}^{new}
$$

6. And then we compute the impulse **j** by the  $v_i^{new}$ and $v_i$ (where $I$ is the inertia matrix of the bunny in the reference state):

$$
K = \dfrac{1}{M} * \mathbb{I} - (Rr_i)^* I^{-1} (Rr_i)^*
$$

$$
j = K^{-1}(v_i^{new} - v_i)
$$

Here we use $r^*$ to represent the cross product matrix of *r*:

$$
r^* =
\begin{bmatrix}
0 & -r_z & r_y\\
r_z & 0 & -r_x\\
-r_y & r_x & 0
\end{bmatrix}
$$

That is to say:

$$
r \times q =
\begin{bmatrix}
r_y * q_z - r_z * q_y\\
r_z * q_x - r_x * q_z\\
r_x * q_y - r_y * q_y\\
\end{bmatrix}
 = \
\begin{bmatrix}
0 & -r_z & r_y\\
r_z & 0 & -r_x\\
-r_y & r_x & 0\\
\end{bmatrix}
\begin{bmatrix}
q_x\\
q_y\\
q_z\\
\end{bmatrix}
= r^*q
$$


7. Finally, we update the *v* and *w* using impuse computed from the former step:

$$
v = v + \dfrac{1}{M}j
$$

$$
w = w + I^{-1}(Rr_i\times j)
$$


### Shape Matching Method

Here is the final result of shape matching method:
![shape_matching](lab1/results/shape_matching.gif)

Here are steps to implement this method:

1. Update each vertex independently:

$$
f_i = Force(x_i, v_i)
$$

$$
v_i = v_i + \Delta t * m_i^{-1}f_i
$$

$$
y_i = x_i + \Delta t v_i
$$

2. Computing the rigid transformation matrix according to the the vertices' new position, here we use polar composition to get rotation matrix $R$ from $A$, as here only rotation exists, no shearing and stretching:

$$
c = \dfrac{1}{N} \sum_{i = 0}^{n - 1} y_i
$$

$$
A = (\sum_{i = 0}^{n-1}(y_i - c)r_i^T)(\sum_{i = 0}^{n - 1}r_ir_i^T)^{-1}
$$

$$
R = Polar(A)
$$

3. Finally update very vertex:

$$
v_i = (c + Rr_i - x_i) / \Delta t
$$

$$
x_i = c + R r_i
$$

A difference between this method and the first one is that, to tranform the bunny, the first method is modifying the bunny's position and orientation, however, In this method, the bunny's poisition and orientation are set to identity, it modifies bunny vertices position for every frame, which maybe a little clumsy and time consuming. another side effect that is prone to ignore is that the shading is wrong caused by not updating the normal of all vertices. so you may add several lines of code in the Update_Mesh method to solve this problem. here are the code snippet, note how the normals are updated. (caution: do not apply the translation to the normal)

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



## Cloth Simulation

In this assignment, I have implemented cloth simulation in a numerical stable fashion by two different approaches, one is implicit   solver, the other is position-based dynamics.

### Implicit Cloth Solver

Here are the steps to implement this approach:

1. Setup the initial status. we traverse all the vertices, and apply damping to the velocity and then calculate $\tilde x_i$:

$$
v_i *= damping
$$

$$
\tilde x_i = x_i + \Delta t v_i
$$

2. Compute the gradient of the objective function. In this lecture, to solve the implicit equation, we convert it to an *argmin* value problem, here are how to calculate the gradient:

$$
g = \dfrac{1}{\Delta t^2}M(x - \tilde x) - f(x)
$$

As you can see, this formula includes two parts, the first part can be calculated vertex by vertex, the second part can be solved by looping through all the springs and compting the force of the two vertices the spring connects.

3. Use Newton's method to solve the non-linear optimization problem, as the Hessian matrix is complicated to construct, here we use a simplified one by considering Hessian matrix as a diagonal one. here it is:

$$
x_i \leftarrow x_i - ({\dfrac{1}{\Delta t^2} + 4k})^{-1}g_i
$$

4. Handle sphere collision. first we traverse all the vertices, and check every one whether it intersects with the sphere, if not, we are done, if yes, we apply impulse-based method as follows:

$$
v_i \leftarrow + \dfrac{1}{\Delta t}(c + r \dfrac{x_i - c}{||x_i - c||} - x_i)
$$

$$
x_i \leftarrow c + r \dfrac {x_i - c}{||x_i - c||}
$$

the central idea here is just pushing the vertex from the current position to the boundary of the sphere along the direction from the center of the sphere to the current position, as long as computing the related velocity.

Here is the final result:

![implicit_model](lab2/results/implicit_model.gif)

### Position-Based Dynamics (PBD)

In this implementation, there are several steps as follows:

1. Setup the initial status, update the velocity and position of very vertex independently, just as a particle system. the damping and gravity should be considered here.

2. Strain limiting. apply the spring constrain to every vertex, that is to say, change the position of every vertex accordingly, by looping through all the springs and accumulating the spring constrain to the two vertices that it connects. here we use two temporary arrays *sumx* and *sumn* to store the sums of vertex position updates and vertex count updates.

$$
sumx_i \leftarrow sumx_i + \dfrac{1}{2}(x_i + x_j + L_e \dfrac{x_i - x_j}{||x_i - x_j||})
$$

$$
sumn_i \leftarrow sumn_i + 1
$$

$$
sumx_j \leftarrow sumx_j + \dfrac{1}{2}(x_i + x_j - L_e \dfrac{x_i - x_j}{||x_i - x_j||})
$$

$$
sumn_j \leftarrow sumn_j + 1
$$

3. Finally update each vertex as:

$$
v_i \leftarrow v_i + \dfrac{1}{\Delta t}(\dfrac{0.2x_i + sumx_i}{0.2 + sumn_i} - x_i)
$$

$$
x_i \leftarrow \dfrac{0.2 x_i + sumx_i}{0.2 + sumn_i}
$$

Note that we should execute the last two steps multiple times to ensure that edge lengths are well preserved. Otherwise, you may see overly stretching artifacts.

Here is the final result:

![PBD_model](lab2/results/PBD_model.gif)



## Bouncy House

In this assignment, we implement the finite element method for the elastic body simulation. Here are the steps:

1. Construct edge matrices. In the Start function, we construct the edge matrices for every tetrahedron in the reference state. since these matrices is unchanged over the period of the simuation, we compute these only once at the start time. here is the edge matrix (where $X_{ij}$ represents the vector from the position  of vertex *i* to vertex *j*  of the tetrahedron respectively in the initial state):
   
$$
\begin {bmatrix}
X_{01} & X_{02} & X_{03}
\end {bmatrix}
$$

2. Initial setup. In the _Update function, we write the simulation of every vertex as a particle system, applying damping to every vertex, and initialize every vertex's force as gravity. 

3. Compute deformation gradient, where $x_{ij}$ denotes the vector from the position of vertex *i* to vertex *j* of the tetrahedron in the current state:

$$
F = 
\begin {bmatrix}
x_{01} & x_{02} & x_{03}
\end {bmatrix}
\begin {bmatrix}
X_{01} & X_{02} & X_{03}
\end {bmatrix}
^{-1}
$$

4. Compute green strain:

$$
G = \dfrac{1}{2}(F^TF - I)
$$

5. Compute second Piola-Kirchhoff stress:

$$
S = 2s_1G + s_0 tr(G) I
$$

6. Compute the forces for the four vertices, here $A_{ref}$ is the volume of the tetrahedron in the reference state:

$$
\begin {bmatrix}
f_1 & f_2 & f_3
\end {bmatrix}
=\
-A^{ref}FS
\begin {bmatrix}
X_{01} & X_{02} & X_{03}
\end {bmatrix}
^{-T}
$$

   According to Newton's third law, we can get $f_0$ by:

$$
f_0 = -f_1 - f_2 - f_3
$$

7. Apply Laplacian smoothing. Laplacian smoothing is crucial to numerical stability. without it, this simulation is easy to blow up. To do it, for every evertex, sum up all of its neighbor's velocity, and blend it into its own velocity.


The final result is shown as follows:

![bouncy house](lab3/results/bouncy_house.gif)



## Pool Ripples

Here are the steps to implement this assignment:

1. Basic setup. we construct the water surface as a *size x size* planar mesh. every vertex of the mesh has a height related to it.

2. Ripples. The height of every vertex is updated as the following formula, here Neumann boundary conditions used on the boundary of the area.

$$
h_{i, j}^{new} \leftarrow h_{i, j} + (h_{i, j} - h_{i, j}^{old}) * damping + (h_{i - 1, j} + h_{i + 1, j} + h_{i, j - 1} + h_{i, j + 1} - 4h_{i, j}) * rate
$$

3. One way coupling from block to water. We use conjugate gradient (CG) method to solve the Poisson equation **Av = b**, where **A** is the masked Laplacian matrix (label area of blocks) and **b = (new_h - low_h) / rate**, **new_h** and **low_h** represents the current height computed from step 2 and the deired height in the masked area. after solving **Av = b**, we have a virtual height **v** corresponding to every vertex. then we get the final height of every vertex by (here $\gamma$ is used to diminish wave effect):

$$
h_{i, j}^{new} \leftarrow h_{i, j}^{new} + (v_{i - 1, j} + v_{i + 1, j} + v_{i, j - 1} + v_{i, j + 1}) * \gamma * rate
$$
   
After implementing the previous several steps, we get the final result:

![pool ripples](lab4/results/pool_ripples.gif)

???			
