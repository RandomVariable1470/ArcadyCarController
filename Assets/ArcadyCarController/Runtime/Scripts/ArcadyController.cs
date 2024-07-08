using UnityEngine;

namespace Arcady
{
    public class ArcadyController : MonoBehaviour
    {
        [SerializeField] private InputReader inputReader;
        [Space(5f)]
        [SerializeField] private Transform[] frontWheels;
        [SerializeField] private Transform[] rearWheels;
        [SerializeField] private Transform carBody;
        
        // Car Properties
        [SerializeField] private float maxSpeed;
        [SerializeField] private float accelerationForce;
        [SerializeField] private float turnTorque;
        [SerializeField] private float brakeForce;
        [Space(5f)]
        [SerializeField] private AnimationCurve accelerationCurve;
        [SerializeField] private AnimationCurve turnCurve;
        [SerializeField] private AnimationCurve driftingCurve;
        [Space(5f)] 
        [SerializeField] private Transform accelerationPoint;
        [Space(5f)]
        [SerializeField] private float driftForce;
        [SerializeField] private float driftControl;
        
        // Aerodynamics
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

        // Visuals
        [SerializeField] private float sidewaysTilt;
        [SerializeField] private float frontTilt;
        [SerializeField] private float sidewaysTiltSpeed;
        [Space(5f)] 
        [SerializeField] private float wheelRotationSpeed;
        [SerializeField] private float wheelSteeringSmoothness;
        [Space(5f)]
        [SerializeField] private AudioSource engineAudioSource;
        [SerializeField] private AudioSource driftAudioSource;
        [SerializeField, Range(0f, 1f)] private float minEnginePitch;
        [SerializeField, Range(1f, 3f)] private float maxEnginePitch;
        [Space(5f)]
        [SerializeField] private TrailRenderer[] driftTrails;
        
        // Ground Check
        [SerializeField] private Transform groundCheck;
        [SerializeField] private float groundCheckDistance;
        
        private Vector3 _originalCenterOfMass;
        private Vector3 _carVelocity;

        private Vector3 _lateralVelocity;

        private float _accelerationNetForce;
        private float _steeringNetForce;
        private float _steerAngle;
        private float _previousSteerAngle;
        private float _breakVelocity;
        
        private float _groundDrag;
        private float _airDrag;
        
        private float _targetFrontTiltAngle;
        private float _targetSideTiltAngle;
        private float _targetFrontTiltAngleVelocity;

        private bool _freezeSteering;
        
        private Rigidbody _carRb;

        private RaycastHit _groundRaycastHit;
        private Coroutine _frontAndBackTiltCoroutine;

        private const float DriftThreshold = 15f;

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
            ApplyVisuals();
        }

        private void FixedUpdate()
        {
            _carVelocity = carBody.InverseTransformDirection(_carRb.velocity);
            _lateralVelocity = Vector3.Project(_carRb.velocity, carBody.right);
            
            AdjustCenterOfMass();
            
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
    
            _accelerationNetForce = accelerationCurve.Evaluate(_carVelocity.magnitude / maxSpeed) * accelerationForce;
            _steeringNetForce = turnCurve.Evaluate(Mathf.Abs(_carVelocity.magnitude / maxSpeed)) * Mathf.Sign(_carVelocity.z) * turnTorque;

            if (inputReader.Brake > 0.1f)
            {
                _carRb.constraints = RigidbodyConstraints.FreezeRotationX;
        
                var newZ = Mathf.SmoothDamp(_carRb.velocity.z, 0, ref _breakVelocity,IsDrifting() ? 50f : 10f);
                _carRb.velocity = new Vector3(_carRb.velocity.x, _carRb.velocity.y, newZ);
            }
            else
            {
                _steerAngle = inputReader.Move.x * _steeringNetForce;
                _carRb.constraints = RigidbodyConstraints.None;
        
                _carRb.AddForceAtPosition(carBody.forward * (_accelerationNetForce * inputReader.Move.y), accelerationPoint.position, ForceMode.Acceleration);
                if (_carVelocity.magnitude / maxSpeed > 0.1f) _carRb.AddTorque(carBody.up * _steerAngle, ForceMode.Acceleration);
            }
        }
        
        private void HandleDrifting()
        {
            if (!IsDrifting()) return;

            float driftForceFactor = driftingCurve.Evaluate(_carVelocity.magnitude / maxSpeed);
            Vector3 driftDirection = Quaternion.AngleAxis(30 * -Mathf.Sign(inputReader.Move.x), Vector3.up) * carBody.forward;
            _carRb.AddForce(driftDirection * (driftForce * driftForceFactor), ForceMode.Acceleration);

            float controlFactor = Mathf.Lerp(1f, driftControl, _carVelocity.magnitude / maxSpeed);
            _carRb.AddForce(-_carRb.velocity * (1f - controlFactor), ForceMode.Acceleration);
        }
        
        private void ApplyAirborneMovement()
        {
            _carRb.drag = _airDrag;

            float airborneSteerAngle = inputReader.Move.x * _steeringNetForce * 0.5f; 
            _carRb.AddTorque(carBody.up * airborneSteerAngle, ForceMode.Acceleration);
        }

        #endregion

        #region Vehicle Dynamics

        private void SidewaysDrag()
        {
            float dragMagnitude = -_lateralVelocity.magnitude * inputReader.Brake > 0.1f ? brakingCoefficient : dragCoefficient;
            
            Vector3 dragForce = _lateralVelocity.normalized * dragMagnitude;
            _carRb.AddForceAtPosition(dragForce, _carRb.worldCenterOfMass, ForceMode.Acceleration);
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
            float rotationAngle = Mathf.Abs(_carVelocity.z / maxSpeed) * Time.deltaTime * wheelRotationSpeed;
                
            foreach (var wheel in frontWheels)
            {
                wheel.localRotation = Quaternion.Slerp(wheel.localRotation, Quaternion.Euler(wheel.localRotation.eulerAngles.x, 
                    _steerAngle, wheel.localRotation.eulerAngles.z), Time.deltaTime);
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
            return Mathf.Abs(_carVelocity.x) > DriftThreshold && Mathf.Abs(inputReader.Move.x) > 0.1f && Mathf.Abs(inputReader.Move.y) > 0.1f && IsGrounded() && _carVelocity.magnitude / maxSpeed > 0.1f;
        }
    }
}