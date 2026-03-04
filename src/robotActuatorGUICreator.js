import * as THREE from 'three';
import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
import { _actuatorSolver, _IKSolver, _getWorldDistance } from './robotCommon.js';

function getActuatorLimit(robot, actuatorName) {
	for (let i = 0; i < robot.actuators.length; i++) {
		if (robot.actuators[i].name === actuatorName) {
			return {
				lower: robot.actuators[i].limit.lower,
				upper: robot.actuators[i].limit.upper,
			};
		}
	}

	return { lower: 0, upper: 1 };
}

function resolveLegacyEndEffectorModel(robot) {
	return (
		robot?.robotModel?.children?.[0]?.children?.[0]?.children?.[0]?.children?.[0]?.children?.[0]?.children?.[1] ??
		null
	);
}

function createLegacyIkTarget(scene) {
	const EETargetPostion = new THREE.Vector3(-0.9381922791167425, 5.5904616136711915, 0);
	const EETargetAngle = 0.9792894692982548 + 1.044677752928888 + 1.162990281672878;

	const EEJointTargetGeo = new THREE.SphereGeometry(0.3, 16, 8);
	const EEJointTargetMat = new THREE.MeshPhongMaterial({ color: 0xffff00, transparent: true, opacity: 0.4 });
	const EEJointTargetMesh = new THREE.Mesh(EEJointTargetGeo, EEJointTargetMat);
	EEJointTargetMesh.isMineRobotOverlay = true;
	EEJointTargetMesh.position.copy(EETargetPostion);
	scene.add(EEJointTargetMesh);
	EEJointTargetMesh.add(new THREE.AxesHelper(1));

	const EETargetGeo = new THREE.BoxGeometry(5, 0.25, 0.5);
	EETargetGeo.applyMatrix4(new THREE.Matrix4().makeTranslation(5 / 2, 0, 0));
	const EETargetMesh = new THREE.Mesh(EETargetGeo, EEJointTargetMat);
	EEJointTargetMesh.add(EETargetMesh);
	EEJointTargetMesh.rotation.set(0, 0, EETargetAngle);

	return { targetMesh: EEJointTargetMesh, targetAngle: EETargetAngle };
}

function robotActuatorGUICreator(robot, scene) {
	const gui = new GUI();
	gui.title('MineRobot');

	const actuatorFolder = gui.addFolder('Actuator Lengths');
	const actuatorsTopo = robot.actuatorsTopo;

	for (const key in actuatorsTopo) {
		if (!Object.prototype.hasOwnProperty.call(actuatorsTopo, key)) {
			continue;
		}

		const actuatorTopo = actuatorsTopo[key];
		const limits = getActuatorLimit(robot, actuatorTopo.name);

		const actuatorState = {
			length: _getWorldDistance(actuatorTopo.rodJointModel, actuatorTopo.tubeJointModel),
		};

		const slider = actuatorFolder
			.add(actuatorState, 'length', limits.lower, limits.upper)
			.name(actuatorTopo.name)
			.onChange((value) => {
				_actuatorSolver(robot, actuatorTopo.name, value);
				actuatorState.length = _getWorldDistance(actuatorTopo.rodJointModel, actuatorTopo.tubeJointModel);
			});

		slider.listen();
	}

	const endEffectorParentModel = resolveLegacyEndEffectorModel(robot);
	if (!endEffectorParentModel) {
		console.warn('[MineRobot] Legacy end-effector path is missing. IK controls are disabled.');
		return gui;
	}

	const { targetMesh: EEJointTargetMesh, targetAngle: EETargetAngle } = createLegacyIkTarget(scene);
	const ikFolder = gui.addFolder('Inverse Kinematics');

	const targetPositionGUIObject = {
		x: -0.9381922791167425,
		y: 5.5904616136711915,
		z: 0,
	};
	const targetAngleGUIObject = {
		angle: EETargetAngle,
	};

	ikFolder.add(targetPositionGUIObject, 'x', -2, 2).onChange((value) => {
		EEJointTargetMesh.position.x = value;
	});

	ikFolder.add(targetPositionGUIObject, 'y', 2, 10).onChange((value) => {
		EEJointTargetMesh.position.y = value;
	});

	const zSlider = ikFolder.add(targetPositionGUIObject, 'z', -1, 1).onChange((value) => {
		EEJointTargetMesh.position.z = value;
	});
	zSlider.disable();

	ikFolder.add(targetAngleGUIObject, 'angle', 0, 2 * Math.PI).onChange((value) => {
		EEJointTargetMesh.rotation.z = value;
	});

	ikFolder
		.add(
			{
				solveIK: () => {
					EEJointTargetMesh.updateMatrixWorld(true);
					_IKSolver(robot, endEffectorParentModel, EEJointTargetMesh.matrixWorld);
				},
			},
			'solveIK',
		)
		.name('Solve IK');

	return gui;
}

export { robotActuatorGUICreator };
