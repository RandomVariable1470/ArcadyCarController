using UnityEngine;

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
        [SerializeField] private float decelerationSpeed = 20f;
        [SerializeField] private float accelerationForce= 100f;
        [SerializeField] private float decelerationForce= 50f;
        [SerializeField] private float turnTorque = 50f;
        [Space(5f)]
        [SerializeField] private AnimationCurve accelerationCurve;
        [SerializeField] private AnimationCurve turnCurve;
        [SerializeField] private AnimationCurve reversingCurve;
        [SerializeField] private AnimationCurve driftingCurve;
        [Space(5f)] 
        [SerializeField] private Transform accelerationPoint;
        [Space(5f)]
        [SerializeField] private float driftForce = 30f;
        [SerializeField] private float driftControl = 0.5f;
        [field: SerializeField] public float DriftSteerThreshold { get; private set; } = 15f;
        
        [Header("Aerodynamics")]
        [SerializeField, Range(50f, 150f)] private float downForce = 100f;
        [SerializeField, Range(5f, 30f)] private float sideGrip = 15f;
        [Space(5f)]
        [SerializeField] private float maxBodyTiltAngle = 20f;
        [Space(5f)]
        [SerializeField] private Transform centerOfMassAirborne;
        [Space(5f)]
        [SerializeField] private float gravity = 15f;
        [Space(5f)]
        [SerializeField] private float dragCoefficient = 1f;
        [SerializeField] private float brakingCoefficient = 0.6f;

        [Header("Visuals")]
        [SerializeField] private float sidewaysTilt = 15f;
        [SerializeField] private float frontTilt = -5f;
        [SerializeField] private float sidewaysTiltSpeed = 7.5f;
        [Space(5f)] 
        [SerializeField] private float wheelRotationSpeed = 30f;

        [Space(5f)] [SerializeField] private AudioSource engineAudioSource;
        [SerializeField] private AudioSource driftAudioSource;
        [SerializeField, Range(0f, 1f)] private float minEnginePitch = 0.5f;
        [SerializeField, Range(1f, 3f)] private float maxEnginePitch = 3f;

        [Header("Ground Check")]
        [SerializeField] private Transform groundCheck;
        [SerializeField] private float groundCheckDistance;
        
        private Vector3 _originalCenterOfMass;
        private Vector3 _carVelocity;

        private Vector3 _lateralVelocity;

        private float _accelerationSpeed;
        private float _decelerationSpeed;
        private float _accelerationNetForce;
        private float _decelerationNetForce;
        private float _steeringNetForce;
        private float _steerAngle;
        private float _breakVelocity;
        
        private float _targetFrontTiltAngle;
        private float _targetSideTiltAngle;
        private float _targetFrontTiltAngleVelocity;
        
        private Rigidbody _carRb;

        private RaycastHit _groundRaycastHit;
        private Coroutine _frontAndBackTiltCoroutine;

        private void OnEnable()
        {
            inputReader.Enable();
        }

        private void Start()
        {
            _carRb = GetComponent<Rigidbody>();
            _originalCenterOfMass = _carRb.centerOfMass;
        }
        
        private void OnDisable()
        {
            inputReader.Disable();
        }

        private void Update()
        {
            IsGrounded();
            ApplyVisuals();
            AdjustCenterOfMass();
        }

        private void FixedUpdate()
        {
            _carVelocity = transform.InverseTransformDirection(_carRb.velocity);
            _accelerationSpeed = _carVelocity.magnitude / maxSpeed;
            _decelerationSpeed = _carVelocity.magnitude / decelerationSpeed;
            _lateralVelocity = Vector3.Project(_carRb.velocity, transform.right);
            
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
            AccelerationAndDeceleration();
            Braking();
            Steering();
            
            void AccelerationAndDeceleration()
            {
                _accelerationNetForce = accelerationCurve.Evaluate(_accelerationSpeed) * accelerationForce * 1000f * Time.fixedDeltaTime;
                _decelerationNetForce = reversingCurve.Evaluate(_decelerationSpeed) * decelerationForce * 1000f * Time.fixedDeltaTime;
                
                if (inputReader.Move.y > 0.1f)
                {
                    _carRb.AddForceAtPosition(transform.forward * _accelerationNetForce, accelerationPoint.position, ForceMode.Acceleration);
                }
                else if (inputReader.Move.y < -0.1f)
                {
                    _carRb.AddForceAtPosition(-transform.forward * _decelerationNetForce, accelerationPoint.position, ForceMode.Acceleration);
                }
            }
            
            void Braking()
            {
                if (inputReader.Brake > 0.1f)
                {
                    _carRb.constraints = RigidbodyConstraints.FreezeRotationX;
                    var newZ = Mathf.SmoothDamp(_carRb.velocity.z, 0, ref _breakVelocity, 1f);
                    _carRb.velocity = new Vector3(_carRb.velocity.x, _carRb.velocity.y, newZ);
                }
            }

            void Steering()
            {
                _steeringNetForce = Mathf.Clamp01(turnCurve.Evaluate(Mathf.Abs(_accelerationSpeed))) * Mathf.Sign(_carVelocity.z) * turnTorque * 1000f * Time.fixedDeltaTime;
                _steerAngle = inputReader.Move.x * _steeringNetForce;
                if (_accelerationSpeed > 0.1f)
                {
                    _carRb.AddTorque(carBody.up * _steerAngle);  
                }  
            }
        }
        
        private void HandleDrifting()
        {
            if (!IsDrifting()) return;

            float driftForceFactor = driftingCurve.Evaluate(_accelerationSpeed);
            Vector3 driftDirection = Quaternion.AngleAxis(30 * -Mathf.Sign(inputReader.Move.x), Vector3.up) * transform.forward;
            _carRb.AddForce(driftDirection * (driftForce * driftForceFactor), ForceMode.Acceleration);

            float controlFactor = Mathf.Lerp(1f, driftControl, _accelerationSpeed);
            _carRb.AddForce(-_carRb.velocity * (1f - controlFactor), ForceMode.Acceleration);
        }
        
        private void ApplyAirborneMovement()
        {
            _carRb.AddForce(-carBody.up * (gravity * _carRb.mass));
            
            float airborneSteerAngle = (inputReader.Move.x * _steeringNetForce) / 2f; 
            _carRb.AddTorque(carBody.up * airborneSteerAngle);
        }

        #endregion

        #region Vehicle Dynamics

        private void SidewaysDrag()
        {
            if (_carVelocity.magnitude > 1f)
            {
                float dragMagnitude = -_lateralVelocity.magnitude * inputReader.Brake > 0.1f ? brakingCoefficient : dragCoefficient;
            
                Vector3 dragForce = _lateralVelocity.normalized * dragMagnitude;
                _carRb.AddForceAtPosition(dragForce, _carRb.centerOfMass);
            }
        }
        
        private void ApplyNormalGroundTilt()
        {
            float tiltAmount = Mathf.Clamp(_lateralVelocity.magnitude / maxSpeed, 0, 1) * Mathf.Sign(inputReader.Move.x) * maxBodyTiltAngle;
            
            Quaternion targetRotation = Quaternion.FromToRotation(_carRb.transform.up, _groundRaycastHit.normal) * _carRb.transform.rotation;
            targetRotation = Quaternion.AngleAxis(tiltAmount, transform.forward) * targetRotation;

            _carRb.MoveRotation(Quaternion.Slerp(_carRb.rotation, targetRotation, Time.fixedDeltaTime));
        }

        private void ApplyDownForce()
        {
             float speedFactor = Mathf.Clamp01(_carRb.velocity.magnitude / maxSpeed);
             float lateralG = Mathf.Abs(Vector3.Dot(_carRb.velocity, carBody.right));
             float downForceFactor = Mathf.Max(speedFactor, lateralG / sideGrip);
             
            _carRb.AddForce(-carBody.up * (downForce * _carRb.mass * downForceFactor));
        }

        private void AdjustCenterOfMass()
        {
            _carRb.centerOfMass = IsGrounded() ? _originalCenterOfMass : centerOfMassAirborne.localPosition;
        }

        #endregion

        #region Visuals

        private void ApplyVisuals()
        {
            WheelVisuals();
            BodyTilting();
            AudioEffect();
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
                    wheelRotationSpeed * inputReader.Move.x, wheel.localRotation.eulerAngles.z), wheelRotationSpeed * Time.deltaTime);
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
        
        #endregion
        
        public bool IsGrounded()
        {
            bool grounded = Physics.Raycast(groundCheck.position, -transform.up, out _groundRaycastHit, groundCheckDistance);
            Debug.DrawLine(groundCheck.position, grounded ? _groundRaycastHit.point : groundCheck.position + -groundCheck.up * groundCheckDistance, grounded ? Color.red : Color.green);
            return grounded;
        }

        private bool IsDrifting()
        {
            return Mathf.Abs(_carVelocity.x) > DriftSteerThreshold && IsGrounded() && Mathf.Abs(_carVelocity.z) / maxSpeed < maxSpeed - 10f;
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.red;
            Gizmos.DrawRay(groundCheck.position, -groundCheck.up * groundCheckDistance);
        }
    }
}