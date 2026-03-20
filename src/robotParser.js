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

function getVec3Length(value) {
	const [x, y, z] = toVec3Array(value, [0, 0, 0]);
	return Math.sqrt(x * x + y * y + z * z);
}

function buildRevoluteJointGraph(jointDatas) {
	const revoluteJoints = jointDatas.filter((joint) => joint.type === 'revolute');
	const outgoing = new Map();
	const edgeMap = new Map();
	const links = new Set();

	revoluteJoints.forEach((joint) => {
		links.add(joint.parent);
		links.add(joint.child);

		if (!outgoing.has(joint.parent)) {
			outgoing.set(joint.parent, []);
		}
		outgoing.get(joint.parent).push(joint);

		if (!outgoing.has(joint.child)) {
			outgoing.set(joint.child, []);
		}

		const edgeKey = `${joint.parent}->${joint.child}`;
		if (!edgeMap.has(edgeKey)) {
			edgeMap.set(edgeKey, []);
		}
		edgeMap.get(edgeKey).push(joint);
	});

	return { revoluteJoints, outgoing, edgeMap, links };
}

function findStronglyConnectedComponents(jointDatas) {
	const { outgoing, links } = buildRevoluteJointGraph(jointDatas);
	const indexMap = new Map();
	const lowLinkMap = new Map();
	const stack = [];
	const onStack = new Set();
	const components = [];
	let index = 0;

	function strongConnect(linkName) {
		indexMap.set(linkName, index);
		lowLinkMap.set(linkName, index);
		index += 1;
		stack.push(linkName);
		onStack.add(linkName);

		for (const joint of outgoing.get(linkName) ?? []) {
			const nextLink = joint.child;
			if (!indexMap.has(nextLink)) {
				strongConnect(nextLink);
				lowLinkMap.set(
					linkName,
					Math.min(lowLinkMap.get(linkName), lowLinkMap.get(nextLink)),
				);
			}
			else if (onStack.has(nextLink)) {
				lowLinkMap.set(
					linkName,
					Math.min(lowLinkMap.get(linkName), indexMap.get(nextLink)),
				);
			}
		}

		if (lowLinkMap.get(linkName) === indexMap.get(linkName)) {
			const component = [];
			while (stack.length > 0) {
				const popped = stack.pop();
				onStack.delete(popped);
				component.push(popped);
				if (popped === linkName) {
					break;
				}
			}

			if (component.length > 1) {
				components.push(component);
			}
		}
	}

	for (const linkName of links) {
		if (!indexMap.has(linkName)) {
			strongConnect(linkName);
		}
	}

	return components;
}

function detectBidirectionalFourBarLoops(jointDatas) {
	const { outgoing, edgeMap } = buildRevoluteJointGraph(jointDatas);
	const components = findStronglyConnectedComponents(jointDatas);
	const loops = [];
	const seen = new Set();

	components.forEach((component) => {
		const componentSet = new Set(component);
		const groundCandidates = componentSet.has('ground')
			? ['ground']
			: componentSet.has('base_link')
				? ['base_link']
				: component;

		groundCandidates.forEach((groundLink) => {
			for (const j0 of outgoing.get(groundLink) ?? []) {
				if (!componentSet.has(j0.child) || j0.child === groundLink) {
					continue;
				}
				const l1 = j0.child;

				for (const j1 of outgoing.get(l1) ?? []) {
					if (!componentSet.has(j1.child)) {
						continue;
					}

					const l2 = j1.child;
					if (l2 === groundLink || l2 === l1) {
						continue;
					}

					for (const j2 of outgoing.get(l2) ?? []) {
						if (!componentSet.has(j2.child)) {
							continue;
						}

						const l3 = j2.child;
						if (l3 === groundLink || l3 === l1 || l3 === l2) {
							continue;
						}

						const drivenClosingJoints = edgeMap.get(`${l3}->${groundLink}`) ?? [];
						const groundClosingJoints = edgeMap.get(`${groundLink}->${l3}`) ?? [];

						if (drivenClosingJoints.length === 0 || groundClosingJoints.length === 0) {
							continue;
						}

						groundClosingJoints.forEach((groundClosingJoint) => {
							drivenClosingJoints.forEach((drivenClosingJoint) => {
								const key = [
									j0.name,
									j1.name,
									j2.name,
									groundClosingJoint.name,
									drivenClosingJoint.name,
								].sort().join('|');

								if (seen.has(key)) {
									return;
								}
								seen.add(key);

								loops.push({
									links: [groundLink, l1, l2, l3],
									pathJoints: [j0, j1, j2],
									closingJoints: {
										ground: groundClosingJoint,
										driven: drivenClosingJoint,
									},
								});
							});
						});
					}
				}
			}
		});
	});

	return loops;
}

function detectLegacyFourBarLoops(jointDatas) {
	const { revoluteJoints, outgoing } = buildRevoluteJointGraph(jointDatas);
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
					const groundLink = links.includes('ground')
						? 'ground'
						: links.includes('base_link')
							? 'base_link'
							: links[0];
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
	if (loop.closingJoints) {
		const groundAnchor = toVec3Array(loop.closingJoints.ground.origin_translation, [0, 0, 0]);
		const drivenAnchor = toVec3Array(loop.closingJoints.driven.origin_translation, [0, 0, 0]);

		return {
			name: `fourbar_auto_${index}`,
			type: 'four-bar',
			joint: loop.pathJoints[2].name,
			ground: loop.links[0],
			ground_closing_joint: loop.closingJoints.ground.name,
			driven_closing_joint: loop.closingJoints.driven.name,
			gound_offset: groundAnchor,
			length: getVec3Length(drivenAnchor),
			init_angle: 1.2,
			removedJointNames: [
				loop.closingJoints.ground.name,
				loop.closingJoints.driven.name,
			],
		};
	}

	const closingJoint = loop.joints[3];
	const groundOffset = toVec3Array(closingJoint.origin_translation, [0, 0, 0]);

	return {
		name: `fourbar_auto_${index}`,
		type: 'four-bar',
		joint: loop.joints[2].name,
		ground: loop.links[0],
		gound_offset: groundOffset,
		length: getVec3Length(groundOffset),
		init_angle: 1.2,
		removedJointNames: [closingJoint.name],
	};
}

function getRemovedJointNamesForFourBarConstraint(constraintData) {
	if (constraintData.type !== 'four-bar') {
		return [];
	}

	const removedJointNames = [];
	if (constraintData.ground_closing_joint) {
		removedJointNames.push(constraintData.ground_closing_joint);
	}
	if (constraintData.driven_closing_joint) {
		removedJointNames.push(constraintData.driven_closing_joint);
	}

	return removedJointNames;
}

function getJointDataByName(jointDatas, jointName) {
	for (let i = 0; i < jointDatas.length; i++) {
		if (jointDatas[i].name === jointName) {
			return jointDatas[i];
		}
	}

	return null;
}

function resolveFourBarConstraintGeometry(constraintData, allJointDatas) {
	const groundClosingJointData = constraintData.ground_closing_joint
		? getJointDataByName(allJointDatas, constraintData.ground_closing_joint)
		: null;
	const drivenClosingJointData = constraintData.driven_closing_joint
		? getJointDataByName(allJointDatas, constraintData.driven_closing_joint)
		: null;

	let groundOffset = constraintData.gound_offset
		? toVec3Array(constraintData.gound_offset, [0, 0, 0])
		: null;
	if (!groundOffset && groundClosingJointData) {
		groundOffset = toVec3Array(groundClosingJointData.origin_translation, [0, 0, 0]);
	}
	if (!groundOffset && drivenClosingJointData) {
		groundOffset = toVec3Array(drivenClosingJointData.origin_translation, [0, 0, 0]);
	}

	let length = Number(constraintData.length);
	if (!Number.isFinite(length)) {
		if (drivenClosingJointData) {
			length = getVec3Length(drivenClosingJointData.origin_translation);
		}
		else if (groundOffset) {
			length = getVec3Length(groundOffset);
		}
	}

	return {
		groundOffset: groundOffset ?? [0, 0, 0],
		length,
		groundClosingJoint: constraintData.ground_closing_joint ?? null,
		drivenClosingJoint: constraintData.driven_closing_joint ?? null,
	};
}

function buildRedundantActuatorGroups(actuators) {
	const adjacency = new Map();

	actuators.forEach((actuator) => {
		adjacency.set(actuator.name, new Set());
	});

	actuators.forEach((actuator) => {
		(actuator.redundants || []).forEach((redundantName) => {
			if (!adjacency.has(redundantName)) {
				return;
			}

			adjacency.get(actuator.name).add(redundantName);
			adjacency.get(redundantName).add(actuator.name);
		});
	});

	const groups = [];
	const groupMap = new Map();
	const representativeMap = new Map();
	const visited = new Set();

	actuators.forEach((actuator) => {
		if (visited.has(actuator.name)) {
			return;
		}

		const stack = [actuator.name];
		const members = [];

		while (stack.length > 0) {
			const currName = stack.pop();
			if (visited.has(currName)) {
				continue;
			}

			visited.add(currName);
			members.push(currName);

			for (const nextName of adjacency.get(currName) || []) {
				if (!visited.has(nextName)) {
					stack.push(nextName);
				}
			}
		}

		const group = {
			name: `redundant_group_${groups.length + 1}`,
			representative: members[0],
			actuators: members,
		};

		groups.push(group);
		members.forEach((memberName) => {
			groupMap.set(memberName, group);
			representativeMap.set(memberName, group.representative);
		});
	});

	return { groups, groupMap, representativeMap };
}

function buildSolverActuators(robot) {
	const solverActuators = [];
	const solverMap = new Map();
	const actuatorSolverNameMap = new Map();

	robot.redundantActuatorGroups.forEach((group) => {
		const representativeActuator = robot.actuatorMap.get(group.representative);
		if (!representativeActuator) {
			return;
		}

		const hasRedundancy = group.actuators.length > 1;
		const solverName = `solver_${group.name}`;
		const solverActuator = {
			name: solverName,
			group: group.name,
			sourceActuator: representativeActuator.name,
			tube_parent: representativeActuator.tube_parent,
			rod_parent: representativeActuator.rod_parent,
			tube_offset: new _Translation(
				representativeActuator.tube_offset.x,
				representativeActuator.tube_offset.y,
				hasRedundancy ? 0 : representativeActuator.tube_offset.z,
			),
			rod_offset: new _Translation(
				representativeActuator.rod_offset.x,
				representativeActuator.rod_offset.y,
				hasRedundancy ? 0 : representativeActuator.rod_offset.z,
			),
		};

		solverActuators.push(solverActuator);
		solverMap.set(solverName, solverActuator);

		group.actuators.forEach((actuatorName) => {
			actuatorSolverNameMap.set(actuatorName, solverName);
		});
	});

	return { solverActuators, solverMap, actuatorSolverNameMap };
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
	let autoFourBarConstraints = [];
	const removedJointNames = new Set();

	const loops = detectBidirectionalFourBarLoops(allJointDatas);
	const autoLoops = loops.length > 0 ? loops : detectLegacyFourBarLoops(allJointDatas);
	autoFourBarConstraints = autoLoops.map((loop, idx) => buildAutoFourBarConstraint(loop, idx + 1));
	autoFourBarConstraints.forEach((constraint) => {
		(constraint.removedJointNames ?? []).forEach((jointName) => {
			removedJointNames.add(jointName);
		});
	});

	const effectiveJointDatas = allJointDatas.filter(
		(jointData) => !removedJointNames.has(jointData.name),
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

	const effectiveConstraints = autoFourBarConstraints.map((constraint) => {
		const { removedJointNames: _removedJointNames, ...rest } = constraint;
		return rest;
	});

	effectiveConstraints.forEach(constraintData => {
		let constraint;
		if (constraintData.type === "four-bar") {
			const {
				groundOffset,
				length,
				groundClosingJoint,
				drivenClosingJoint,
			} = resolveFourBarConstraintGeometry(constraintData, allJointDatas);

			constraint = new _FourBarConstraint(
				constraintData.name,
				constraintData.type,
				constraintData.joint,
				constraintData.ground,
				groundOffset,
				length,
				Number(constraintData.init_angle ?? 1.0),
				groundClosingJoint,
				drivenClosingJoint,
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
					actuatorData.redundants || [],
					actuatorData.tube_visual,
					actuatorData.rod_visual,
				);
			}

			robot.actuators.push(actuator);
			robot.actuatorMap.set(actuator.name, actuator);
		});
	}

	const { groups, groupMap, representativeMap } = buildRedundantActuatorGroups(robot.actuators);
	robot.redundantActuatorGroups = groups;
	robot.actuatorRedundantGroupMap = groupMap;
	robot.actuatorRepresentativeMap = representativeMap;
	const { solverActuators, solverMap, actuatorSolverNameMap } = buildSolverActuators(robot);
	robot.solverActuators = solverActuators;
	robot.solverActuatorMap = solverMap;
	robot.actuatorSolverNameMap = actuatorSolverNameMap;

	return robot;

}

export { robotParser };
