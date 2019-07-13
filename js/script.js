console.log('hello');

const stick = document.querySelector('.stick__input');
const plane = document.querySelector('.plane__input');

let inMin = +document.querySelector('[name=inMin]').value;
let inCent = +document.querySelector('[name=inCent]').value;
let inMax = +document.querySelector('[name=inMax]').value;

let outMin = +document.querySelector('[name=outMin]').value;
let outCent = +document.querySelector('[name=outCent]').value;
let outMax = +document.querySelector('[name=outMax]').value;
const inv = document.querySelector('[name=inv]');

const graph = document.getElementById("canvas");
const elev = document.querySelector('.elev');

let rate;
let exp = +document.querySelector('[name=exp]').value/10;


const exponent = [];
let koefUp, koefDown, koefUpOut, koefDownOut;
let output = 0;

const showGraph = () => {
  let ctx = graph.getContext('2d');
  const koefWidth = (outMax - outMin)/graph.width;
  const koefHeight= (outMax - outMin) / graph.height;

  ctx.fillStyle = '#fff';
  ctx.fillRect(0, 0, graph.width, graph.height);

  for (let i = inMin; i <= inMax; i+=40) {
    ctx.fillStyle = '#000000';
    ctx.fillRect(i/koefWidth, graph.height-2 - (calc(i)-outMin)/koefHeight,2,2);
  }
}

const init = () => {

  inMin = +document.querySelector('[name=inMin]').value;
  inCent = +document.querySelector('[name=inCent]').value;
  inMax = +document.querySelector('[name=inMax]').value;

  outMin = +document.querySelector('[name=outMin]').value;
  outCent = +document.querySelector('[name=outCent]').value;
  outMax = +document.querySelector('[name=outMax]').value;

  rate = document.querySelector('[name=rate]').value;

  exp = (50/100) * document.querySelector('[name=exp]').value;
  koefUp = inMax-inCent;
  koefDown = inCent - inMin;
  koefUpOut = outMax - outCent;
  koefDownOut = outCent - outMin;

  stick.min = inMin;
  stick.max = inMax;
  stick.value = inCent;
  showGraph();
}

const calc = (input) => {
  let tmp;


  if (input >= inCent) {
    tmp = (input - inCent) / koefUp;
    tmp = ((exp**tmp)-1)/(exp-1);
    tmp = (tmp * (koefUpOut * (rate/100))) + outCent;
  } else {
    tmp = (inCent - input) / koefDown;
    tmp = ((exp**tmp)-1)/(exp-1);
    tmp = outCent - (tmp * (koefDownOut * (rate/100)));
  }

  console.log(tmp);

  return tmp;
}


stick.addEventListener('input', (e) => {
  const input = e.currentTarget.value;

  output = calc(input);

  elev.style = `transform: rotate(${(output - 1520) * 0.1}deg);`;

  plane.value = output;

})

stick.addEventListener('mouseup', () => {
  stick.value = inCent;
  elev.style = `transform: rotate(${(output - 1520) * 0.1}deg);`;
  plane.value = calc(inCent);
})


button.addEventListener('click', () => {
  init();
  console.log(exponent);
  console.log(koefUp, koefDown);
})
