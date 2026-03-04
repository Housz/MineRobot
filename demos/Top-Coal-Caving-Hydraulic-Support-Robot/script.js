import { createMineVisApp } from '../../src/app.js';

const canvas = document.querySelector('#c');
const app = createMineVisApp({ canvas });
app.loadRobotFromUrl('./hydraulic_support.json');
