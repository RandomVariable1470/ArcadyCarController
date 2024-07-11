using UnityEngine;
using Cinemachine;

namespace Arcady
{
    public class ArcadyController : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private InputReader inputReader;
        [Space(5f)]
        [SerializeField] private Transform[] frontWheels;
        [SerializeField] private Transform[] rearWheels;
        [SerializeField] private Transform carBody;
        
        [Header("Car Properties")]
        [SerializeField] private float maxSpeed = 70f;
        [SerializeField] private float accelerationForce= 100f;
        [SerializeField] private float turnTorque = 50f;
        [Space(5f)]
        [SerializeField] private AnimationCurve accelerationCurve;
        [SerializeField] private AnimationCurve turnCurve;
        [SerializeField] private AnimationCurve driftingCurve;
        [Space(5f)] 
        [SerializeField] private Transform accelerationPoint;
        [Space(5f)]
        [SerializeField] private float driftForce = 30f;
        [SerializeField] private float driftControl = 0.5f;
        [SerializeField] private float driftSteerThreshold = 15f;
        
        [Header("Aerodynamics")]
        [SerializeField, Range(50f, 150f)] private float downForce = 100f;
        [SerializeField, Range(5f, 30f)] private float sideGrip = 15f;
        [Space(5f)]
        [SerializeField, Range(0.1f, 30f)] private float gravity = 5f;
        [Space(5f)]
        [SerializeField] private float maxBodyTiltAngle = 20f;
        [Space(5f)]
        [SerializeField] private float minSpeedThreshold = 20f;
        [SerializeField] private float maxSpeedThreshold = 60f;
        [SerializeField] private float adjustCenterOfMassOffset = -0.5f;
        [SerializeField] private Transform centerOfMassGrounded;
        [SerializeField] private Transform centerOfMassAirborne;
        [Space(5f)]
        [SerializeField] private float dragCoefficient = 1f;
        [SerializeField] private float brakingCoefficient = 0.6f;

        [Header("Visuals")]
        [SerializeField] private float sidewaysTilt = 15f;
        [SerializeField] private float frontTilt = -5f;
        [SerializeField] private float sidewaysTiltSpeed = 7.5f;
        [Space(5f)] 
        [SerializeField] private float wheelRotationSpeed = 30f;
        [Space(5f)]
        [SerializeField] private AudioSource engineAudioSource;
        [SerializeField] private AudioSource driftAudioSource;
        [SerializeField, Range(0f, 1f)] private float minEnginePitch = 0.5f;
        [SerializeField, Range(1f, 3f)] private float maxEnginePitch = 3f;
        [Space(5f)]
        [SerializeField] private TrailRenderer[] driftTrails;
        
        [Header("Vehicle Camera")]
        [SerializeField] private CinemachineVirtualCamera vehicleCamera;
        [Space(5f)]
        [SerializeField] private float maxShakeAmplitude = 0.25f;
        [SerializeField] private float minShakeAmplitude = 0.05f;
        [SerializeField] private float maxShakeFrequency = 0.2f;
        [SerializeField] private float minShakeFrequency = 0.025f;
        
        [Header("Ground Check")]
        [SerializeField] private Transform groundCheck;
        [SerializeField] private float groundCheckDistance;
        
        private Vector3 _originalCenterOfMass;
        private Vector3 _carVelocity;

        private Vector3 _lateralVelocity;

        private float _speed;
        private float _accelerationNetForce;
        private float _steeringNetForce;
        private float _steerAngle;
        private float _breakVelocity;

        private float _groundDrag;
        private float _airDrag;
        
        private float _targetFrontTiltAngle;
        private float _targetSideTiltAngle;
        private float _targetFrontTiltAngleVelocity;
        
        private Rigidbody _carRb;

        private RaycastHit _groundRaycastHit;
        private Coroutine _frontAndBackTiltCoroutine;

        private const float TerminalVelocity = 63.25f; // This terminal velocity is for a 150 kg car gravity as 9.81f
                                                       // p 1.225f cd is 0.3
                                                       // and an is 4 these are apporx values
                                                       // but u can adjust bases on ur needs and put ur
                                                       // terminal velocity value here

        private void OnEnable()
        {
            inputReader.Enable();
        }

        private void Start()
        {
            _carRb = GetComponent<Rigidbody>();
            _originalCenterOfMass = centerOfMassGrounded.localPosition;
            
            _groundDrag = gravity / TerminalVelocity;
            _carRb.drag = _groundDrag;

            _airDrag = _carRb.drag * 10f;
        }
        
        private void OnDisable()
        {
            inputReader.Disable();
        }

        private void Update()
        {
            IsGrounded();
            //CameraAdjustments();
            ApplyVisuals();
            AdjustCenterOfMass();
        }

        private void FixedUpdate()
        {
            _carVelocity = carBody.InverseTransformDirection(_carRb.velocity);
            _speed = _carVelocity.magnitude / maxSpeed;
            _lateralVelocity = Vector3.Project(_carRb.velocity, carBody.right);
            
            if (IsGrounded())
            {
                HandleGroundedMovement();
                HandleDrifting();
                ApplyDownForce();
                ApplyNormalGroundTilt();
                SidewaysDrag();
            }
            else
            {
                ApplyAirborneMovement();
            }
        }

        #region Vehicle Movement

        private void HandleGroundedMovement()
        {
            _carRb.drag = _groundDrag;

            _accelerationNetForce = accelerationCurve.Evaluate(_speed) * accelerationForce;
            _steeringNetForce = turnCurve.Evaluate(Mathf.Abs(_speed)) * Mathf.Sign(_carVelocity.z) * turnTorque;

            if (inputReader.Brake > 0.1f)
            {
                _carRb.constraints = RigidbodyConstraints.FreezeRotationX;
                var newZ = Mathf.SmoothDamp(_carRb.velocity.z, 0, ref _breakVelocity, 1f);
                _carRb.velocity = new Vector3(_carRb.velocity.x, _carRb.velocity.y, newZ);
            }
            else
            {
                _steerAngle = inputReader.Move.x * _steeringNetForce;
                _carRb.constraints = RigidbodyConstraints.None;

                _carRb.AddForceAtPosition(carBody.forward * (_accelerationNetForce * inputReader.Move.y), accelerationPoint.position, ForceMode.Acceleration);
                if (_speed > 0.1f) _carRb.AddTorque(carBody.up * _steerAngle, ForceMode.Acceleration);
            }
        }

        
        private void HandleDrifting()
        {
            if (!IsDrifting()) return;

            float driftForceFactor = driftingCurve.Evaluate(_speed);
            Vector3 driftDirection = Quaternion.AngleAxis(30 * -Mathf.Sign(inputReader.Move.x), Vector3.up) * carBody.forward;
            _carRb.AddForce(driftDirection * (driftForce * driftForceFactor), ForceMode.Acceleration);

            float controlFactor = Mathf.Lerp(1f, driftControl, _speed);
            _carRb.AddForce(-_carRb.velocity * (1f - controlFactor), ForceMode.Acceleration);
        }
        
        private void ApplyAirborneMovement()
        {
            _carRb.drag = _airDrag;

            float airborneSteerAngle = inputReader.Move.x * _steeringNetForce; 
            _carRb.AddTorque(carBody.up * airborneSteerAngle, ForceMode.Acceleration);
        }

        #endregion

        #region Vehicle Dynamics

        private void SidewaysDrag()
        {
            if (_carVelocity.magnitude > 1)
            {
                // Calculate the friction angle based on the car's tilt
                float frictionAngle = (-Vector3.Angle(transform.up, Vector3.up) / 90f) + 1;

                // Calculate the stabilizing force
                Vector3 stabilizingForce = Vector3.ProjectOnPlane(transform.right, Vector3.up) * (dragCoefficient * frictionAngle * 100 * -_carVelocity.normalized.x);

                // Apply the stabilizing force at the car's center of mass
                _carRb.AddForceAtPosition(stabilizingForce, _carRb.worldCenterOfMass, ForceMode.Acceleration);
            }
        }

        
        private void ApplyNormalGroundTilt()
        {
            float tiltAmount = Mathf.Clamp(_lateralVelocity.magnitude / maxSpeed, 0, 1) * Mathf.Sign(inputReader.Move.x) * maxBodyTiltAngle;
            
            Quaternion targetRotation = Quaternion.FromToRotation(_carRb.transform.up, _groundRaycastHit.normal) * _carRb.transform.rotation;
            targetRotation = Quaternion.AngleAxis(tiltAmount, carBody.forward) * targetRotation;

            _carRb.MoveRotation(Quaternion.Slerp(_carRb.rotation, targetRotation, Time.fixedDeltaTime));
        }

        private void ApplyDownForce()
        {
             float speedFactor = Mathf.Clamp01(_carRb.velocity.magnitude / maxSpeed);
             float lateralG = Mathf.Abs(Vector3.Dot(_carRb.velocity, transform.right));
             float downForceFactor = Mathf.Max(speedFactor, lateralG / sideGrip);
             
            _carRb.AddForce(-transform.up * (downForce * _carRb.mass * downForceFactor));
        }

        private void AdjustCenterOfMass()
        {
            if (IsGrounded())
            {
                float speedFactor = Mathf.InverseLerp(minSpeedThreshold, maxSpeedThreshold, _carRb.velocity.magnitude);
                float verticalInputFactor = Mathf.Abs(inputReader.Move.y) > 0.1f ? Mathf.Sign(inputReader.Move.y) : 0f;
                Vector3 centerOfMassAdjustment = new Vector3(
                    0f, 
                    0f, 
                    verticalInputFactor * speedFactor * adjustCenterOfMassOffset
                );
                _carRb.centerOfMass = _originalCenterOfMass + centerOfMassAdjustment;
            }
            else
            {
                _carRb.centerOfMass = centerOfMassAirborne.localPosition;
            }
        }

        #endregion

        #region Vehicle Camera

        private void CameraAdjustments()
        {
            float shakeAmplitude = Mathf.Lerp(minShakeAmplitude, maxShakeAmplitude, _speed);
            float shakeFrequency = Mathf.Lerp(minShakeFrequency, maxShakeFrequency, _speed);
            
            CinemachineBasicMultiChannelPerlin noise = vehicleCamera.GetCinemachineComponent<CinemachineBasicMultiChannelPerlin>();
            noise.m_AmplitudeGain = shakeAmplitude;
            noise.m_FrequencyGain = shakeFrequency;
        }

        #endregion

        #region Visuals

        private void ApplyVisuals()
        {
            WheelVisuals();
            BodyTilting();
            AudioEffect();
            VFXEffects();
        }
        
        private void BodyTilting()
        {
            if (_carVelocity.z > 1)
            {
                _targetFrontTiltAngle = Mathf.Lerp(0f, frontTilt, Mathf.Abs(_carVelocity.z / maxSpeed));
                _targetSideTiltAngle = sidewaysTilt * inputReader.Move.x;

                if (Mathf.Abs(inputReader.Move.x) < 0.1f)
                {
                    _targetFrontTiltAngle = Mathf.Lerp(_targetFrontTiltAngle, 0f, Mathf.Abs(_carVelocity.z / maxSpeed) * 10f);
                    _targetSideTiltAngle = 0;
                }
            }
            else
            {
                _targetFrontTiltAngle = Mathf.Lerp(_targetFrontTiltAngle, 0f, Mathf.Abs(_carVelocity.z / maxSpeed));
                _targetSideTiltAngle = 0;
            }

            Quaternion targetRotation = Quaternion.Euler(_targetFrontTiltAngle, carBody.localRotation.eulerAngles.y, _targetSideTiltAngle);
            carBody.localRotation = Quaternion.RotateTowards(carBody.localRotation, targetRotation, Time.deltaTime * sidewaysTiltSpeed);
        }
        
        private void WheelVisuals()
        {
            float rotationAngle = Mathf.Abs(_carVelocity.z / maxSpeed) * wheelRotationSpeed;
                
            foreach (var wheel in frontWheels)
            {
                wheel.localRotation = Quaternion.Slerp(wheel.localRotation, Quaternion.Euler(wheel.localRotation.eulerAngles.x, 
                    _steerAngle, wheel.localRotation.eulerAngles.z), wheelRotationSpeed * Time.deltaTime);
            }
            foreach (var wheel in rearWheels)
            {
                wheel.Rotate(Vector3.right, rotationAngle);
            }
        }
        
        private void AudioEffect()
        {
            engineAudioSource.pitch = Mathf.Lerp(minEnginePitch, maxEnginePitch, Mathf.Abs(_carVelocity.z) / maxSpeed);
            driftAudioSource.mute = !IsDrifting();
        }
        
        private void VFXEffects()
        {
            foreach (var driftTrail in driftTrails)
            {
                driftTrail.emitting = IsDrifting();
            }
        }

        #endregion
        
        private bool IsGrounded()
        {
            bool grounded = Physics.Raycast(groundCheck.position, -transform.up, out _groundRaycastHit, groundCheckDistance);
            Debug.DrawLine(groundCheck.position, grounded ? _groundRaycastHit.point : groundCheck.position + -groundCheck.up * groundCheckDistance, grounded ? Color.red : Color.green);
            return grounded;
        }

        private bool IsDrifting()
        {
            return Mathf.Abs(_carVelocity.x) > driftSteerThreshold && IsGrounded() && _speed > 0.5f;
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.red;
            Gizmos.DrawRay(groundCheck.position, -groundCheck.up * groundCheckDistance);
        }
    }
}