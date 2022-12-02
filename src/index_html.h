String INDEX_HTML = R"(<!DOCTYPE html>
<html>

<head>
  <meta http-equiv='Cache-Control' content='no-cache, no-store, must-revalidate' />
  <meta http-equiv='Pragma' content='no-cache' />
  <meta http-equiv='Expires' content='0' />
  <title>{NAME}</title>
  <link rel='stylesheet' href='https://unpkg.com/onsenui/css/onsenui.css'>
  <link rel='stylesheet' href='https://unpkg.com/onsenui/css/onsen-css-components.min.css'>
  <script src='https://unpkg.com/onsenui/js/onsenui.min.js'></script>
  <script src='https://unpkg.com/jquery/dist/jquery.min.js'></script>
  <script>
    var cversion = '{VERSION}';
    var wsUri = 'ws://' + location.host + ':81/';
    var repo = 'motor-on-roller-blind-ws';

    window.fn = {};
    window.fn.open = function () {
      var menu = document.getElementById('menu');
      menu.open();
    };

    window.fn.load = function (page) {
      var content = document.getElementById('content');
      var menu = document.getElementById('menu');
      content.load(page)
        .then(menu.close.bind(menu)).then(setActions());
    };

    var gotoPos = function (percent) {
      doSend(percent);
    };

    var setActions = function () {
      doSend('{"action": "update"}');
      $.get('https://api.github.com/repos/speham/' + repo + '/releases', function (data) {
        if (data.length > 0 && data[0].tag_name !== cversion) {
          $('#cversion').text(cversion);
          $('#nversion').text(data[0].tag_name);
          $('#update-card').show();
        }
      });

      setTimeout(function () {
        $('#arrow-close1').on('click', function () { $('#setrange1').val(100); gotoPos('{"id": 1, "action": "auto", "value": 100}'); });
        $('#arrow-open1').on('click', function () { $('#setrange1').val(0); gotoPos('{"id": 1, "action": "auto", "value": 0}'); });
        $('#setrange1').on('change', function () { gotoPos('{"id": 1, "action": "auto", "value": ' + $('#setrange1').val() + '}') });
        $('#setspeed').on('change', function () { doSend('{"id": 1, "action": "setspeed", "value": ' + $('#setspeed').val() + '}') });
        $('#arrow-up-man1').on('click', function () { doSend('{"id": 1, "action": "manual", "value": -1}') });
        $('#arrow-down-man1').on('click', function () { doSend('{"id": 1, "action": "manual", "value": 1}') });
        $('#arrow-stop-man1').on('click', function () { doSend('{"id": 1, "action": "manual", "value": 0}') });
        $('#set-start1').on('click', function () { doSend('{"id": 1, "action": "start", "value":0}') });
        $('#set-max1').on('click', function () { doSend('{"id": 1, "action": "max", "value": 0}') });

      }, 200);
    };
    $(document).ready(function () {
      setActions();
    });

    var websocket;
    var timeOut;
    function retry() {
      clearTimeout(timeOut);
      timeOut = setTimeout(function () {
        websocket = null; init();
      }, 5000);
    };
    function init() {
      ons.notification.toast({ message: 'Connecting...', timeout: 1000 });
      try {
        websocket = new WebSocket(wsUri);
        websocket.onclose = function () { };
        websocket.onerror = function (evt) {
          ons.notification.toast({ message: 'Cannot connect to device', timeout: 2000 });
          retry();
        };
        websocket.onopen = function (evt) {
          ons.notification.toast({ message: 'Connected to device', timeout: 2000 });
          setTimeout(function () { doSend('{"action": "update"}'); }, 1000);
        };
        websocket.onclose = function (evt) {
          ons.notification.toast({ message: 'Disconnected. Retrying', timeout: 2000 });
          retry();
        };
        websocket.onmessage = function (evt) {
          try {
            var msg = JSON.parse(evt.data);

            if (typeof msg.position1 !== 'undefined') {
              $('#pbar1').attr('value', msg.position1);
            };
            if (typeof msg.set1 !== 'undefined') {
              $('#setrange1').val(msg.set1);
              $('#setrange1').attr('value', msg.set1);
            };
            if (typeof msg.speed1 !== 'undefined') {
              $('#setspeed').attr('value', msg.speed1);
            };
            if (typeof msg.switch1 !== 'undefined') {
              var state1 = msg.switch1;
              if (state1 == true) { $('#button-sw1').prop('checked', true); }
              else { $('#button-sw1').prop('checked', false); }
            };
            if (typeof msg.switch2 !== 'undefined') {
              var state2 = msg.switch2;
              if (state2 == true) { $('#button-sw2').prop('checked', true); }
              else { $('#button-sw2').prop('checked', false); }
            };
          } catch (err) { }
        };
      } catch (e) {
        ons.notification.toast({ message: 'Cannot connect to device. Retrying...', timeout: 2000 });
        retry();
      };
    };
    function doSend(msg) {
      if (websocket && websocket.readyState == 1) {
        websocket.send(String(msg));
      }
    };
    function wipeSettings() {
      if (confirm("Are you really sure to wipe ALL settings?")) {
        $.ajax({
          type: "POST",
          url: "/reset",
          contentType: 'application/json',
        })
      }
    }
    window.addEventListener('load', init, false);
    window.onbeforeunload = function () {
      if (websocket && websocket.readyState == 1) {
        websocket.close();
      };
    };
  </script>
</head>

<body>

  <ons-splitter>
    <ons-splitter-side id='menu' side='left' width='220px' collapse swipeable>
      <ons-page>
        <ons-list>
          <ons-list-item onclick='fn.load("home.html")' tappable>
            Home
          </ons-list-item>
          <ons-list-item onclick='fn.load("settings.html")' tappable>
            Settings
          </ons-list-item>
          <ons-list-item onclick='fn.load("about.html")' tappable>
            About
          </ons-list-item>
          <ons-list-item onclick='wipeSettings()' tappable>
            Wipe settings
          </ons-list-item>
        </ons-list>
      </ons-page>
    </ons-splitter-side>
    <ons-splitter-content id='content' page='home.html'></ons-splitter-content>
  </ons-splitter>

  <template id='home.html'>
    <ons-page>
      <ons-toolbar>
        <div class='left'>
          <ons-toolbar-button onclick='fn.open()'>
            <ons-icon icon='md-menu'></ons-icon>
          </ons-toolbar-button>
        </div>
        <div class='center'>
          {NAME}
        </div>
      </ons-toolbar>
      <ons-card>
        <div class='title'>Adjust position</div>
        <div class='content'>
          <p>Move the slider to the wanted position or use the arrows to open/close to the max positions</p>
        </div>
        <!-- --------------------1 -->
        <ons-row>
          <ons-col width='40px' style='text-align: center; line-height: 31px;'>
          </ons-col>
          <ons-col>
            <ons-progress-bar id='pbar1' value='75'></ons-progress-bar>
          </ons-col>
          <ons-col width='40px' style='text-align: center; line-height: 31px;'>
          </ons-col>
        </ons-row>
        <ons-row>
          <ons-col width='40px' style='text-align: center; line-height: 31px;'>
            <ons-icon id='arrow-open1' icon='fa-arrow-up' size='2x'></ons-icon>
          </ons-col>
          <ons-col>
            <ons-range id='setrange1' style='width: 100%;' value='75'></ons-range>
          </ons-col>
          <ons-col width='40px' style='text-align: center; line-height: 31px;'>
            <ons-icon id='arrow-close1' icon='fa-arrow-down' size='2x'></ons-icon>
          </ons-col>
        </ons-row>
      </ons-card>

      <ons-card>
        <div class='title'>Inputs / Motorspeed</div>
        <div class='content'>
          <p></p>
        </div>
        <!-- --------------------settings -->
        <ons-row>
          <ons-col width='40px' style='text-align: center; line-height: 50px;'>
            <ons-icon icon='fa-toggle-on' size='2x'></ons-icon>
          </ons-col>
          <ons-col width='130px' style='text-align: right; line-height: 50px;'>switch 1
            <ons-switch disabled style='vertical-align: middle;' id='button-sw1'></ons-switch>
          </ons-col>
          <ons-col width='130px' style='text-align: center; line-height: 50px;'>switch 2
            <ons-switch disabled style='vertical-align: middle;' id='button-sw2'></ons-switch>
          </ons-col>
          <ons-col width='40px' style='text-align: center; line-height: 50px;'>
          </ons-col>
        </ons-row>

        <ons-row>
          <ons-col width='40px' style='text-align: center; line-height: 31px;'>
            <ons-icon icon='fa-tachometer' size='2x'></ons-icon>
          </ons-col>
          <ons-col>
            <ons-range id='setspeed' style='width: 100%;' value='20'></ons-range>
          </ons-col>
          <ons-col width='40px' style='text-align: center; line-height: 31px;'>
          </ons-col>
        </ons-row>
      </ons-card>


      <ons-card id='update-card' style='display:none'>
        <div class='title'>Update available</div>
        <div class='content'>You are running <span id='cversion'></span> and <span id='nversion'></span> is the latest.
          Go to <a href='https://github.com/speham/motor-on-roller-blind-ws/releases'>the repo</a> to download</div>
      </ons-card>
    </ons-page>
  </template>

  <template id='settings.html'>
    <ons-page>
      <ons-toolbar>
        <div class='left'>
          <ons-toolbar-button onclick='fn.open()'>
            <ons-icon icon='md-menu'></ons-icon>
          </ons-toolbar-button>
        </div>
        <div class='center'>
          Settings
        </div>
      </ons-toolbar>
      <ons-card>
        <div class='title'>Instructions</div>
        <div class='content'>
          <p>
          <ol>
            <li>Use the arrows and stop button to navigate to the top position i.e. the blind is opened</li>
            <li>Click the START button</li>
            <li>Use the down arrow to navigate to the max closed position</li>
            <li>Click the MAX button</li>
            <li>Calibration is completed!</li>
          </ol>
          </p>
        </div>
      </ons-card>
      <!-- ---------------------1 -->
      <ons-card>
        <div class='title'>Main Blind</div>
        <ons-row style='width:100%'>
          <ons-col style='text-align:center'>
            <ons-icon id='arrow-up-man1' icon='fa-arrow-up' size='2x'></ons-icon>
          </ons-col>
          <ons-col style='text-align:center'>
            <ons-icon id='arrow-stop-man1' icon='fa-stop' size='2x'></ons-icon>
          </ons-col>
          <ons-col style='text-align:center'>
            <ons-icon id='arrow-down-man1' icon='fa-arrow-down' size='2x'></ons-icon>
          </ons-col>
          <ons-row style='width:100%'>
            <ons-col style='text-align:center'>
              <ons-button id='set-start1'>Set Start</ons-button>
            </ons-col>
            <ons-col style='text-align:center'>&nbsp;</ons-col>
            <ons-col style='text-align:center'>
              <ons-button id='set-max1'>Set Max</ons-button>
            </ons-col>
          </ons-row>
      </ons-card>
    </ons-page>
  </template>

  <template id='about.html'>
    <ons-page>
      <ons-toolbar>
        <div class='left'>
          <ons-toolbar-button onclick='fn.open()'>
            <ons-icon icon='md-menu'></ons-icon>
          </ons-toolbar-button>
        </div>
        <div class='center'>
          About
        </div>
      </ons-toolbar>
      <ons-card>
        <div class='title'>Motor on a roller blind</div>
        <div class='content'>
          <p>
          <ul>
            <li>3d print files and instructions: <a href='https://www.thingiverse.com/thing:2392856'
                target='_blank'>https://www.thingiverse.com/thing:2392856</a></li>
            <li>Forked from: <a href='https://github.com/nidayand/motor-on-roller-blind-ws'
                target='_blank'>https://github.com/nidayand/motor-on-roller-blind-ws</a></li>
            <li>Current fork on Github: <a href='https://github.com/speham/motor-on-roller-blind-ws'
                target='_blank'>https://github.com/speham/motor-on-roller-blind-ws</a></li>
            <li>Licensed under <a
                href='https://raw.githubusercontent.com/nidayand/motor-on-roller-blind-ws/master/LICENSE'
                target='_blank'>MIT License</a></li>
          </ul>
          </p>
        </div>
      </ons-card>
    </ons-page>
  </template>

</body>

</html>
)";
