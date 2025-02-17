function initialize_webiopi(){
    // webiopiの準備が終わってからstyles.cssを適用する
    applyCustomCss('styles.css');

    // GPIOの状態を監視しない
    webiopi().refreshGPIO(false);

    mCanvas = document.getElementById("canvas");
    mCtx = mCanvas.getContext('2d');

    resize_canvas();

    // Firefoxで画面の回転を検出
    var mqOrientation = window.matchMedia("(orientation: portrait)");
    mqOrientation.addListener(function() {
        resize_canvas();
    });
}

// iOSで画面の回転を検出
window.onorientationchange = function()
{
    var iR = Math.abs( window.orientation );
    if ( iR == 0 || iR == 90 ){
        resize_canvas();
    }
}

var mWidth = 0;
var mHeight = 0;

var mImg1;
var mImg2;

var host = location.host;
var hostname = host.split(":")[0];
var imageURL = 'http://' + hostname + ':' + '9000/?action=snapshot';

var commandID = 0;
var mCount = 0;

var mCanvas;
var mCtx;

var sliderMin = 0;
var sliderMax = 20;
var sliderStep = 1;
var sliderValue = sliderMax/2;

$(function() {
    var sliderHandler1 = function(e, ui){
        var ratio = ui.value/sliderMax;
        // サーボの回転の向きを逆にしたい場合次の行を無効に
        //ratio = 1.0 - ratio;
        webiopi().callMacro("setPCA9685PWM", [1, ratio, commandID++]);
    };
    var sliderHandler0 = function(e, ui){
        var ratio = ui.value/sliderMax;
        // サーボの回転の向きを逆にしたい場合次の行を無効に
        //ratio = 1.0 - ratio;
        webiopi().callMacro("setPCA9685PWM", [0, ratio, commandID++]);
    };

    $( "#slider1" ).slider({
        orientation: "vertical",
        min: sliderMin,
        max: sliderMax,
        step: sliderStep,
        value: sliderValue,
        change: sliderHandler1,
        slide: sliderHandler1
    });
    $( "#slider0" ).slider({
        orientation: "horizontal",
        min: sliderMin,
        max: sliderMax,
        step: sliderStep,
        value: sliderValue,
        change: sliderHandler0,
        slide: sliderHandler0
    });
});

function resize_canvas(){

    if($(window).width() < 4*$(window).height()/3){
        isPortrait = true;
    }else{
        isPortrait = false;
    }

    if(isPortrait){
        mWidth = 0.9*$(window).width();
        mHeight = 3*mWidth/4;
    }else{
        mHeight = 0.9*$(window).height();
        mWidth = 4*mHeight/3;
    }

    mCanvas.width = mWidth;
    mCanvas.height = mHeight;

    $( "#slider1" ).height(mHeight);
    $( "#slider0" ).width(mWidth);

    imageSetup();
}

function imageSetup(){

    mImg1 = new Image();
    mImg2 = new Image();

    mImg1.src = imageURL+'&'+(mCount++);

    mImg1.onload = function() {
        mImg2.src = imageURL+'&'+(mCount++);
        mCtx.drawImage(mImg1, 0, 0, mWidth, mHeight);
    };

    mImg2.onload = function() {
        mImg1.src = imageURL+'&'+(mCount++);
        mCtx.drawImage(mImg2, 0, 0, mWidth, mHeight);
    };
}

function applyCustomCss(custom_css){
    var head = document.getElementsByTagName('head')[0];
    var style = document.createElement('link');
    style.rel = "stylesheet";
    style.type = 'text/css';
    style.href = custom_css;
    head.appendChild(style);
}

