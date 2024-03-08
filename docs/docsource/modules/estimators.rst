.. _observer_module:

=========================
Observer module
=========================
The |project_name| Observer module contains several state (and parameter) observers.
Observers are used to infer states and parameters from measurements. For a more detailed
description of the methods refer to :ref:`the API <observer_autodoc>`.

The Observer module contains the following classes:

- Moving Horizon Estimator (MHE)
- Kalman Filter (KF)
- Extended Kalman Filter (EKF)
- Unscented Kalman Filter (UKF)
- Particle Filter (PF)

-----------------------------------
Moving Horizon Estimator
-----------------------------------
Moving horizon estimation (MHE) is an optimization-based state estimation technique. Utilizing a finite sequence of 
previously observed measurements, which inherently include noise and other inaccuracies, MHE estimates the current 
state of the system. In the toolbox, the class MHE can set a Moving Horizon Estimator where we need to define the following elements:

1. A horizon length.
2. An objective function.
3. constraints (optional)

Horizon length 
------------------------------
To achieve an effective state estimate, the ideal approach involves considering all past measurements from the initial 
time up to the present, leading to what is referred to as a full information estimator. Unfortunately, the computational 
complexity becomes prohibitive as the number of measurements increases. Given this challenge, moving horizon estimation 
uses only a fixed number :math:`N` of past measurements.  At each time step, the optimal estimated state trajectory on the current 
estimation horizon is determined by the past :math:`N` measurements. When a new measurement arrives at the instant :math:`T`, the estimation 
horizon moves forward one time step and the oldest measurement from time :math:`T-N-1`  is discarded. The size of the optimization 
problems remains constant. The moving process is shown in the following figure.

.. image:: ../images/mhe_horizon.png

Thus horizon length is an important parameter in terms of accuracy and computational complexity. The class provides the :code:`horizon` property
to set this parameter.

.. code-block:: python

 mhe = MHE(model)
 mhe.horizon = 10


Objective function
------------------------------
We assume that the current instant is :math:`T` and the horizon is :math:`N`. The objective of MHE is to find an initial state :math:`x_{T - N}` and the disturbance 
sequence :math:`\mathbf{w} = [w_{T - N}, w_{T - N + 1}, \dots, w_{T-1}]^T` so that the objective function is minimized which can be formulated as follows:

.. math::
   &\hspace{5em}\min_{x_{\scriptstyle T-N},\mathbf{w}}\quad\Phi_T\\

The objective function is constructed as follows:

.. math::
 \begin{aligned}
 \hspace{3em}\Phi_T &= Z_{T-N}\left( x_{T-N} \right) + \sum_{k = T - N}^{T - 1} L\left( v_k,w_k\right) \\
       &= \left\| {x_{T-N} - \bar{x}_{T-N}} \right\|_{\Pi^{-1}}^{2} + \sum_{k = T - N}^{T - 1}\left\|v_{k} \right\|_{R^{-1}}^{2}+\left\|w_{k} \right\|_{Q^{-1}}^{2} \\
       &\mathrm{s.t.} \qquad x_{k + 1} = f\left( x_k, u_k \right) + w_k \\
       & \hspace{4em} y_k = h_k\left( x_k, u_k \right) + v_k \\
       & \hspace{3em} x_k \in \mathcal{X}_k, \;w_k \in \mathcal{W}_k, \;v_k \in \mathcal{V}_k
 \end{aligned}


The objective function :math:`\Phi_T` contains two parts, the stage cost function :math:`L\left ( v_k,w_k\right )` and the arrival cost :math:`Z_{T-N}\left ( x_{T-N} \right )`.
The stage cost function :math:`L\left ( v_k,w_k\right ) = \left\|v_{k} \right\|_{R^{-1}}^{2}+\left\|w_{k} \right\|_{Q^{-1}}^{2}` is commonly quadratic and 
contains two parts, namely measurement noise cost :math:`\left\|v_{k} \right\|_{R^{-1}}^{2}` and state noise cost :math:`\left\|w_{k} \right\|_{Q^{-1}}^{2}`, :math:`Q` and :math:`R`
are the covariance matrix of the state noise and measurement noise.
The arrival cost is :math:`Z_{T-N}\left( x_{T-N} \right) = \left\| {x_{T-N} - \bar{x}_{T-N}} \right\|_{\Pi^{-1}}^{2}`.  :math:`\Pi` is the covariance matrix of the initial guess.

The class MHE uses :code:`quad_stage_cost` and :code:`quad_arrival_cost` to construct a quadratic objective function:

:code:`quad_stage_cost.add_state_noise` is to set the state noise cost :math:`\left\|w_{k} \right\|_{Q^{-1}}^{2}` and the parameter :code:`weights` is the confidence matrix of the state noise which is
often set as the inverse of the covariance matrix of the state noise :math:`Q^{-1}`

:code:`quad_stage_cost.add_measurements` is to set the measurement noise cost :math:`\left\|v_{k} \right\|_{R^{-1}}^{2}` and the parameter :code:`weights` is the confidence matrix of the measurement noise which is
often set as the inverse of the covariance matrix of the measurement noise :math:`R^{-1}`

:code:`quad_arrival_cost.add_states` is to set the arrival state cost :math:`Z_{T-N}\left ( x_{T-N} \right )`. The parameter :code:`weights` is the confidence matrix for the knowledge of the initial guess
and the parameter :code:`guess` is set as the initial guess.

For example, if we know the variance of the measurement noise is :math:`0.25^2`, the covariance matrix of the state noise is :math:`\begin{bmatrix} (0.001)^2 & 0 & 0 \\ 0 & (0.001)^2 & 0 \\ 0 & 0 & (0.001)^2 \end{bmatrix}`
the covariance matrix of the initial guess is :math:`\begin{bmatrix} (0.5)^2 & 0 & 0 \\ 0 & (0.5)^2 & 0 \\ 0 & 0 & (0.5)^2 \end{bmatrix}`. The code could be:

.. code-block:: python

  mhe = MHE(model)

  mhe.quad_arrival_cost.add_states(weights=[1/(0.5**2), 1/(0.5**2), 1/(0.5**2)], guess=x0_est)

  mhe.quad_stage_cost.add_measurements(weights=[1/(0.25**2)])

  mhe.quad_stage_cost.add_state_noise(weights=[1/(0.001**2), 1/(0.001**2), 1/(0.001**2)])


Constraints
------------------------------
The class provides a method :code:`set_box_constraints` to set the constraints:

The parameter :code:`x_ub` /:code:`x_lb` is the upper/lower bound of the dynamical state.

The parameter :code:`w_ub` /:code:`w_lb` is the upper/lower bound of the input.

The parameter :code:`p_ub` /:code:`p_lb` is the upper/lower bound of the parameter.

The parameter :code:`z_ub` /:code:`z_lb` is the upper/lower bound of the algebraic state.

For example, if we want to set the constraint that all the add_states(3 dynamical states) must be positive, the code could be :

.. code-block:: python

  mhe = MHE(model)

  mhe.set_box_constraints(x_lb=[0, 0, 0])



Non-uniform sampling intervals
-------------------------------

Multi-rate measurements
-------------------------

-----------------------------------
Kalman Filter
-----------------------------------
The class :class:`~hilo_mpc.KalmanFilter` (alias :class:`~hilo_mpc.KF`) implements the Kalman filter developed by Rudolf E. Kálmán. To set up the Kalman filter you need an already set up :class:`~hilo_mpc.Model` instance. Additionally you might want to supply a plot backend (via the :obj:`plot_backend` keyword argument) in order to visualize the estimation results later on. At the moment only `Matplotlib <https://matplotlib.org/>`_ and `Bokeh <https://bokeh.org/>`_ are supported for plotting. The Kalman filter can be initialized as follows:

.. code-block:: python

    from hilo_mpc import KF


    # Initialize Kalman filter
    kf = KF(model, plot_backend='bokeh')

Required information, like e.g. the model dynamics or the sampling time, will be automatically extracted from the :class:`~hilo_mpc.Model` instance.

-----------------------------------
Extended Kalman Filter
-----------------------------------

-----------------------------------
Unscented Kalman Filter
-----------------------------------

-----------------------------------
Particle Filter
-----------------------------------
