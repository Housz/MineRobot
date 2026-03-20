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

function resolveConfiguredEndEffectorLinkName(robot) {
	const configuredEndEffectorCandidates = {
		'Top Coal Caving Hydraulic Support Robot': ['top'],
		'Shield Type Hydraulic Support Robot': ['top', 'head'],
	};

	const candidates = configuredEndEffectorCandidates[robot?.name];
	if (!candidates || !robot?.linkMap) {
		return null;
	}

	for (const candidate of candidates) {
		if (robot.linkMap.has(candidate)) {
			return candidate;
		}
	}

	return null;
}

function resolveEndEffectorLinkName(robot) {
	const configuredLinkName = resolveConfiguredEndEffectorLinkName(robot);
	if (configuredLinkName) {
		return configuredLinkName;
	}

	const linkNames = new Set((robot?.links || []).map((link) => link.name));
	const parentLinkNames = new Set();
	const rodParentNames = new Set();

	(robot?.joints || []).forEach((joint) => {
		if (linkNames.has(joint.parent)) {
			parentLinkNames.add(joint.parent);
		}
	});

	(robot?.actuators || []).forEach((actuator) => {
		if (linkNames.has(actuator.rod_parent)) {
			rodParentNames.add(actuator.rod_parent);
		}
	});

	const leafLinks = Array.from(linkNames).filter((linkName) => !parentLinkNames.has(linkName));
	const preferredLeaf = leafLinks.find((linkName) => rodParentNames.has(linkName));

	return preferredLeaf ?? leafLinks[0] ?? null;
}

function resolveEndEffectorModel(robot) {
	const endEffectorLinkName = resolveEndEffectorLinkName(robot);
	if (!endEffectorLinkName) {
		return null;
	}

	return robot?.robotModel?.getObjectByName(endEffectorLinkName) ?? null;
}

function getEndEffectorVisualLength(endEffectorLink) {
	const geometry = endEffectorLink?.visual?.geometry;
	if (!geometry) {
		return 5;
	}

	if (geometry.type === 'box') {
		return Math.max(geometry.x ?? 0, 0.25);
	}

	if (geometry.type === 'cylinder') {
		return Math.max(geometry.h ?? 0, 0.25);
	}

	return 5;
}

function createTargetMaterial() {
	return new THREE.MeshPhongMaterial({
		color: 0xffff00,
		transparent: true,
		opacity: 0.25,
		depthTest: false,
		depthWrite: false,
	});
}

function applyVisualTransform(geometry, visual) {
	const visualOriginOrientation = visual?.origin_orientation ?? [0, 0, 0];
	const visualOriginTranslation = visual?.origin_translation ?? [0, 0, 0];

	geometry.rotateX(visualOriginOrientation[0] ?? 0);
	geometry.rotateY(visualOriginOrientation[1] ?? 0);
	geometry.rotateZ(visualOriginOrientation[2] ?? 0);
	geometry.translate(
		visualOriginTranslation[0] ?? 0,
		visualOriginTranslation[1] ?? 0,
		visualOriginTranslation[2] ?? 0,
	);
}

function createTargetVisualMeshFromLink(endEffectorLink, fallbackLength) {
	const visual = endEffectorLink?.visual;
	const geometryConfig = visual?.geometry;

	if (geometryConfig?.type === 'box') {
		const x = geometryConfig.x ?? fallbackLength;
		const y = geometryConfig.y ?? 0.25;
		const z = geometryConfig.z ?? 0.5;
		const geometry = new THREE.BoxGeometry(x, y, z);
		geometry.applyMatrix4(new THREE.Matrix4().makeTranslation(x / 2, 0, 0));
		applyVisualTransform(geometry, visual);

		const targetMesh = new THREE.Mesh(geometry, createTargetMaterial());
		targetMesh.renderOrder = 999;
		return targetMesh;
	}

	if (geometryConfig?.type === 'cylinder') {
		const r1 = geometryConfig.r1 ?? 0.1;
		const r2 = geometryConfig.r2 ?? 0.1;
		const h = geometryConfig.h ?? fallbackLength;
		const geometry = new THREE.CylinderGeometry(r1, r2, h, 16);
		geometry.rotateZ(Math.PI / 2);
		geometry.applyMatrix4(new THREE.Matrix4().makeTranslation(h / 2, 0, 0));
		applyVisualTransform(geometry, visual);

		const targetMesh = new THREE.Mesh(geometry, createTargetMaterial());
		targetMesh.renderOrder = 999;
		return targetMesh;
	}

	const fallbackGeometry = new THREE.BoxGeometry(fallbackLength, 0.25, 0.5);
	fallbackGeometry.applyMatrix4(new THREE.Matrix4().makeTranslation(fallbackLength / 2, 0, 0));
	const fallbackMesh = new THREE.Mesh(
		fallbackGeometry,
		createTargetMaterial(),
	);
	fallbackMesh.renderOrder = 999;
	return fallbackMesh;
}

function createIkTarget(scene, robot, endEffectorLinkName, endEffectorModel) {
	endEffectorModel.updateWorldMatrix(true, true);
	const endEffectorLink = robot?.linkMap?.get(endEffectorLinkName) ?? null;

	const EETargetPostion = new THREE.Vector3();
	endEffectorModel.getWorldPosition(EETargetPostion);

	const worldEuler = new THREE.Euler().setFromRotationMatrix(endEffectorModel.matrixWorld);
	const EETargetAngle = worldEuler.z;
	const targetLength = getEndEffectorVisualLength(endEffectorLink);

	const EEJointTargetGeo = new THREE.SphereGeometry(0.3, 16, 8);
	const EEJointTargetMat = new THREE.MeshPhongMaterial({ color: 0xffff00, transparent: true, opacity: 0.4 });
	const EEJointTargetMesh = new THREE.Mesh(EEJointTargetGeo, EEJointTargetMat);
	EEJointTargetMesh.isMineRobotOverlay = true;
	EEJointTargetMesh.position.copy(EETargetPostion);
	scene.add(EEJointTargetMesh);
	EEJointTargetMesh.add(new THREE.AxesHelper(1));

	const EETargetMesh = createTargetVisualMeshFromLink(endEffectorLink, targetLength);
	EEJointTargetMesh.add(EETargetMesh);
	EEJointTargetMesh.rotation.set(0, 0, EETargetAngle);

	return { targetMesh: EEJointTargetMesh, targetAngle: EETargetAngle };
}

function robotActuatorGUICreator(robot, scene) {
	const gui = new GUI();
	gui.title('MineRobot');

	const actuatorFolder = gui.addFolder('Actuator Lengths');
	const actuatorsTopo = robot.actuatorsTopo;
	const actuatorStates = new Map();
	const actuatorControllers = new Map();

	function refreshActuatorStates() {
		for (const key in actuatorsTopo) {
			if (!Object.prototype.hasOwnProperty.call(actuatorsTopo, key)) {
				continue;
			}

			const actuatorTopo = actuatorsTopo[key];
			const actuatorState = actuatorStates.get(actuatorTopo.name);
			if (!actuatorState) {
				continue;
			}

			actuatorState.length = _getWorldDistance(actuatorTopo.rodJointModel, actuatorTopo.tubeJointModel);
			const controller = actuatorControllers.get(actuatorTopo.name);
			if (controller) {
				controller.updateDisplay();
			}
		}
	}

	for (const key in actuatorsTopo) {
		if (!Object.prototype.hasOwnProperty.call(actuatorsTopo, key)) {
			continue;
		}

		const actuatorTopo = actuatorsTopo[key];
		const limits = getActuatorLimit(robot, actuatorTopo.name);

		const actuatorState = {
			length: _getWorldDistance(actuatorTopo.rodJointModel, actuatorTopo.tubeJointModel),
		};
		actuatorStates.set(actuatorTopo.name, actuatorState);

		const slider = actuatorFolder
			.add(actuatorState, 'length', limits.lower, limits.upper)
			.name(actuatorTopo.name)
			.onChange((value) => {
				_actuatorSolver(robot, actuatorTopo.name, value);
				refreshActuatorStates();
			});

		slider.listen();
		actuatorControllers.set(actuatorTopo.name, slider);
	}

	const endEffectorLinkName = resolveEndEffectorLinkName(robot);
	const endEffectorParentModel = resolveEndEffectorModel(robot);
	if (!endEffectorParentModel) {
		console.warn('[MineRobot] End-effector model is missing. IK controls are disabled.');
		return gui;
	}

	const { targetMesh: EEJointTargetMesh, targetAngle: EETargetAngle } = createIkTarget(
		scene,
		robot,
		endEffectorLinkName,
		endEffectorParentModel,
	);
	const ikFolder = gui.addFolder('Inverse Kinematics');

	const currentEEWorldPosition = new THREE.Vector3();
	endEffectorParentModel.getWorldPosition(currentEEWorldPosition);
	const targetPositionGUIObject = {
		x: currentEEWorldPosition.x,
		y: currentEEWorldPosition.y,
		z: currentEEWorldPosition.z,
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
					refreshActuatorStates();
				},
			},
			'solveIK',
		)
		.name('Solve IK');

	return gui;
}

export { robotActuatorGUICreator };
