import {
	_Robot, _Link, _Joint, _Constraint, _FourBarConstraint,
	_Actuator, _Actuator3, _Limit, _Axis, _Translation,
	_Orientation, _Visual
} from './robotCommon.js'

function toVec3Array(value, fallback = [0, 0, 0]) {
	if (!value) {
		return fallback.slice();
	}

	if (Array.isArray(value) && value.length >= 3) {
		return [Number(value[0]), Number(value[1]), Number(value[2])];
	}

	if (typeof value === 'object') {
		return [Number(value.x ?? 0), Number(value.y ?? 0), Number(value.z ?? 0)];
	}

	return fallback.slice();
}

function rotateArray(arr, startIndex) {
	return arr.map((_, i) => arr[(startIndex + i) % arr.length]);
}

function detectAutoFourBarLoops(jointDatas) {
	const revoluteJoints = jointDatas.filter((joint) => joint.type === 'revolute');
	const outgoing = new Map();

	revoluteJoints.forEach((joint) => {
		if (!outgoing.has(joint.parent)) {
			outgoing.set(joint.parent, []);
		}
		outgoing.get(joint.parent).push(joint);
	});

	const loops = [];
	const seen = new Set();

	for (const j0 of revoluteJoints) {
		const l0 = j0.parent;
		const l1 = j0.child;

		for (const j1 of outgoing.get(l1) ?? []) {
			if (j1.name === j0.name) {
				continue;
			}
			const l2 = j1.child;
			if (l2 === l0 || l2 === l1) {
				continue;
			}

			for (const j2 of outgoing.get(l2) ?? []) {
				if (j2.name === j0.name || j2.name === j1.name) {
					continue;
				}

				const l3 = j2.child;
				if (l3 === l0 || l3 === l1 || l3 === l2) {
					continue;
				}

				for (const j3 of outgoing.get(l3) ?? []) {
					if (j3.name === j0.name || j3.name === j1.name || j3.name === j2.name) {
						continue;
					}
					if (j3.child !== l0) {
						continue;
					}

					const key = [j0.name, j1.name, j2.name, j3.name].sort().join('|');
					if (seen.has(key)) {
						continue;
					}
					seen.add(key);

					const links = [l0, l1, l2, l3];
					const joints = [j0, j1, j2, j3];
					const groundLink = links.includes('base_link') ? 'base_link' : links[0];
					const startIndex = links.indexOf(groundLink);

					loops.push({
						links: rotateArray(links, startIndex),
						joints: rotateArray(joints, startIndex),
					});
				}
			}
		}
	}

	return loops;
}

function buildAutoFourBarConstraint(loop, index) {
	const closingJoint = loop.joints[3];
	const groundOffset = toVec3Array(closingJoint.origin_translation, [0, 0, 0]);

	const drivenLength = Math.sqrt(
		groundOffset[0] * groundOffset[0]
		+ groundOffset[1] * groundOffset[1]
		+ groundOffset[2] * groundOffset[2],
	);

	return {
		name: `fourbar_auto_${index}`,
		type: 'four-bar',
		joint: loop.joints[2].name,
		ground: loop.links[0],
		gound_offset: groundOffset,
		length: drivenLength,
		init_angle: 1.0,
		closingJointName: closingJoint.name,
	};
}

function robotParser(robotJson) {

	let robot = new _Robot(robotJson.name, [], [], [], []);

	robotJson.links.forEach(linkData => {

		if (linkData.name == "world" || linkData.name == "base_link") {
			return;
		}

		let link = new _Link(
			linkData.name,
			linkData.origin_translation,
			linkData.origin_orientation,
			linkData.visual
		);

		robot.links.push(link);
		robot.linkMap.set(link.name, link);

	});

	const allJointDatas = (robotJson.joints || []).filter((jointData) => jointData.name !== 'world_joint');
	const rawConstraints = robotJson.constraints || [];
	const hasExplicitFourBar = rawConstraints.some((constraintData) => constraintData.type === 'four-bar');

	let autoFourBarConstraints = [];
	const autoRemovedJointNames = new Set();

	if (!hasExplicitFourBar) {
		const loops = detectAutoFourBarLoops(allJointDatas);
		autoFourBarConstraints = loops.map((loop, idx) => buildAutoFourBarConstraint(loop, idx + 1));
		autoFourBarConstraints.forEach((constraint) => {
			autoRemovedJointNames.add(constraint.closingJointName);
		});
	}

	const effectiveJointDatas = allJointDatas.filter(
		(jointData) => !autoRemovedJointNames.has(jointData.name),
	);

	effectiveJointDatas.forEach(jointData => {

		let joint = new _Joint(
			jointData.name,
			jointData.type,
			jointData.parent,
			jointData.child,
			jointData.origin_translation,
			jointData.origin_orientation,
			jointData.axis || [0, 0, 0],
			jointData.limit || { "lower": 0, "upper": 1.57 }
		);

		robot.joints.push(joint);
		robot.jointMap.set(joint.name, joint);
	});

	const effectiveConstraints = rawConstraints.concat(
		autoFourBarConstraints.map((constraint) => {
			const { closingJointName, ...rest } = constraint;
			return rest;
		}),
	);

	effectiveConstraints.forEach(constraintData => {
		let constraint;
		if (constraintData.type === "four-bar") {
			constraint = new _FourBarConstraint(
				constraintData.name,
				constraintData.type,
				constraintData.joint,
				constraintData.ground,
				constraintData.gound_offset,
				constraintData.length,
				constraintData.init_angle
			);
		}
		else if (constraintData.type === "triangle-prismatic") {
			constraint = new _Constraint(
				constraintData.name,
				constraintData.type,
				constraintData.jointL,
				constraintData.jointA,
				constraintData.jointB,
				constraintData.jointL_base_angle,
				constraintData.length,
				constraintData.limit
			);
		}
		else {
			return;
		}

		robot.constraints.push(constraint);
		robot.constraintMap.set(constraint.name, constraint);
	});

	if (robotJson.actuators) {
		robotJson.actuators.forEach(actuatorData => {

			let actuator;
			if (actuatorData.type === "actuator") {
				actuator = new _Actuator(
					actuatorData.name,
					actuatorData.type,
					actuatorData.tube_parent,
					actuatorData.rod_parent,
					actuatorData.tube_offset,
					actuatorData.rod_offset,
					actuatorData.limit,
					actuatorData.tube_visual,
					actuatorData.rod_visual,
				);
			}

			robot.actuators.push(actuator);
			robot.actuatorMap.set(actuator.name, actuator);
		});
	}

	return robot;

}

export { robotParser };
