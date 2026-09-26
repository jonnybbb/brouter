'use strict';
const data=window.constructionReport,$=id=>document.getElementById(id),NS='http://www.w3.org/2000/svg';
const ORANGE='#bd530b',BLUE='#007ba4',GREEN='#288559',RED='#bf2847';
let active='rural_lozere_80km_gravel_E';
let observers=[];
function el(tag,text,cls){const n=document.createElement(tag);if(text!==undefined)n.textContent=text;if(cls)n.className=cls;return n;}
function svgEl(tag,attrs){const n=document.createElementNS(NS,tag);for(const[k,v]of Object.entries(attrs))n.setAttribute(k,v);return n;}
function link(text,url){const a=el('a',text);a.href=url;return a;}
function fmt(v,unit='',div=1){return v===null||v===undefined||!Number.isFinite(v)?'unavailable':`${(v/div).toLocaleString(undefined,{maximumFractionDigits:2})}${unit?' '+unit:''}`;}
function pretty(s){return s.replaceAll('_',' ');}
function project(points,origin){const sx=.111195*Math.cos((origin[1]/1e6-90)*Math.PI/180);return points.map(p=>[(p[0]-origin[0])*sx,-(p[1]-origin[1])*.111195]);}
function extent(lines,minWidth=100){let a=Infinity,b=Infinity,c=-Infinity,d=-Infinity;for(const line of lines)for(const[x,y]of line){a=Math.min(a,x);b=Math.min(b,y);c=Math.max(c,x);d=Math.max(d,y);}if(!Number.isFinite(a))return[-50,-50,100,100];return[(a+c)/2,(b+d)/2,Math.max(minWidth,c-a)*1.2,Math.max(minWidth*.6,d-b)*1.2];}
function path(points){return points.map((p,i)=>`${i?'L':'M'}${p[0].toFixed(3)},${p[1].toFixed(3)}`).join(' ');}
function legend(items){const n=el('div',undefined,'legend');for(const[text,color]of items){const s=el('span',text);s.style.setProperty('--color',color);n.append(s);}return n;}
function table(headers,rows){const w=el('div',undefined,'tableWrap'),t=el('table'),h=el('thead'),r=el('tr');headers.forEach(x=>r.append(el('th',x)));h.append(r);t.append(h);const body=el('tbody');for(const row of rows){const tr=el('tr');row.forEach(x=>tr.append(el('td',x)));body.append(tr);}t.append(body);w.append(t);return w;}
function map(caseData,closeup=false){
  const box=el('div'),controls=el('div',undefined,'mapControls'),svg=svgEl('svg',{class:'map',role:'img','aria-label':closeup?'Exact historical defect and captured native road geometry':'Historical and regenerated routes on shared geographic coordinates'}),scale=el('div',undefined,'scaleText');
  const origin=caseData.before.points[0],before=project(caseData.before.points,origin),after=caseData.after?project(caseData.after.points,origin):[];
  const def=caseData.defect,focus=def?project(def.focus,origin):[];
  const full=extent([before,after]),local=def?extent([focus],def.minWidth):full;
  let center=(closeup?local:full).slice(),drag=null,view;
  const grid=svgEl('g',{});svg.append(grid);
  const historical=svgEl('path',{d:path(before),fill:'none',stroke:ORANGE,'stroke-width':closeup?3:4,'vector-effect':'non-scaling-stroke','stroke-linejoin':'round',opacity:.85});svg.append(historical);
  const current=svgEl('path',{d:path(after),fill:'none',stroke:BLUE,'stroke-width':2.3,'vector-effect':'non-scaling-stroke','stroke-linejoin':'round'});if(!closeup)svg.append(current);
  let marker=null;
  if(closeup&&def){
    if(def.native.length)svg.append(svgEl('path',{d:path(project(def.native,origin)),fill:'none',stroke:GREEN,'stroke-width':3,'stroke-dasharray':'7 5','vector-effect':'non-scaling-stroke'}));
    if(def.highlight.length>1)svg.append(svgEl('path',{d:path(project(def.highlight,origin)),fill:'none',stroke:RED,'stroke-width':5,'vector-effect':'non-scaling-stroke'}));
    const markerPoints=def.highlight.length?def.highlight:[def.focus[0]];
    marker=svgEl('path',{d:project(markerPoints,origin).map(p=>`M${p[0]},${p[1]}l0,0`).join(' '),stroke:RED,'stroke-width':10,'stroke-linecap':'round','vector-effect':'non-scaling-stroke'});svg.append(marker);
  }else svg.append(svgEl('path',{d:'M0,0l0,0',stroke:'#162e2b','stroke-width':10,'stroke-linecap':'round','vector-effect':'non-scaling-stroke'}));
  function update(){const rect=svg.getBoundingClientRect(),ratio=(rect.width||900)/(rect.height||480);let w=center[2],h=center[3];if(w/h<ratio)w=h*ratio;else h=w/ratio;view=[center[0]-w/2,center[1]-h/2,w,h];svg.setAttribute('viewBox',view.join(' '));grid.replaceChildren();const raw=w/6,power=10**Math.floor(Math.log10(raw)),step=(raw/power<2?1:raw/power<5?2:5)*power;
    for(let x=Math.floor(view[0]/step)*step;x<view[0]+w;x+=step)grid.append(svgEl('path',{d:`M${x},${view[1]}v${h}`,class:'grid','vector-effect':'non-scaling-stroke'}));
    for(let y=Math.floor(view[1]/step)*step;y<view[1]+h;y+=step)grid.append(svgEl('path',{d:`M${view[0]},${y}h${w}`,class:'grid','vector-effect':'non-scaling-stroke'}));
    const lon=origin[0]/1e6-180+center[0]/(.111195*Math.cos((origin[1]/1e6-90)*Math.PI/180))/1e6,lat=origin[1]/1e6-90-center[1]/.111195/1e6;
    scale.textContent=`Grid ${fmt(step,step>=1000?'km':'m',step>=1000?1000:1)} · centre ${lat.toFixed(5)}°, ${lon.toFixed(5)}° · North ↑ · drag to pan; scroll to zoom`;
  }
  function point(e){const p=svg.createSVGPoint();p.x=e.clientX;p.y=e.clientY;return p.matrixTransform(svg.getScreenCTM().inverse());}
  function zoom(factor,p={x:center[0],y:center[1]}){if(center[2]*factor<1||center[2]*factor>full[2]*20)return;center=[p.x+(center[0]-p.x)*factor,p.y+(center[1]-p.y)*factor,center[2]*factor,center[3]*factor];update();}
  svg.addEventListener('wheel',e=>{e.preventDefault();zoom(e.deltaY>0?1.2:1/1.2,point(e));},{passive:false});
  svg.onpointerdown=e=>{drag=point(e);svg.setPointerCapture(e.pointerId);};svg.onpointermove=e=>{if(!drag)return;const p=point(e);center[0]+=drag.x-p.x;center[1]+=drag.y-p.y;update();};svg.onpointerup=svg.onpointercancel=()=>{drag=null;};
  const fit=el('button',closeup?'Reset close-up':'Whole route'),minus=el('button','−'),plus=el('button','+');fit.onclick=()=>{center=(closeup?local:full).slice();update();};minus.onclick=()=>zoom(1.5);plus.onclick=()=>zoom(1/1.5);controls.append(fit,minus,plus);
  if(!closeup){for(const[label,node]of [['Historical',historical],['Regenerated',current]]){const b=el('button',label);let visible=true;b.setAttribute('aria-pressed','true');b.onclick=()=>{visible=!visible;node.style.display=visible?'':'none';b.setAttribute('aria-pressed',String(visible));};controls.append(b);}}
  box.append(controls,svg,scale);requestAnimationFrame(update);const observer=new ResizeObserver(update);observer.observe(svg);observers.push(observer);return box;
}
function distance(a,b){return Math.hypot((a[0]-b[0])*.111195*Math.cos(((a[1]+b[1])/2e6-90)*Math.PI/180),(a[1]-b[1])*.111195);}
function elevation(routes){const series=routes.map(r=>{let dist=0;return r.points.map((p,i)=>{if(i)dist+=distance(r.points[i-1],p);return[dist,p[2]];});});let maxD=1,lo=Infinity,hi=-Infinity;for(const s of series)for(const[d,e]of s){maxD=Math.max(maxD,d);if(e!==null&&Number.isFinite(e)){lo=Math.min(lo,e);hi=Math.max(hi,e);}}if(!Number.isFinite(lo))return el('p','Elevation unavailable.');lo-=10;hi=Math.max(hi+10,lo+30);const svg=svgEl('svg',{viewBox:'0 0 1000 230',class:'elevation',role:'img','aria-label':'Elevation versus distance for historical and regenerated routes'});
  series.forEach((s,i)=>{let d='',active=false;for(const[x,y]of s){if(y===null){active=false;continue;}d+=`${active?'L':'M'}${55+x/maxD*920},${190-(y-lo)/(hi-lo)*170}`;active=true;}svg.append(svgEl('path',{d,fill:'none',stroke:i?BLUE:ORANGE,'stroke-width':2,'stroke-dasharray':i?'none':'6 3'}));});
  for(const[x,y,t]of [[3,20,`${Math.round(hi)} m`],[3,190,`${Math.round(lo)} m`],[55,220,'0 km'],[880,220,`${(maxD/1000).toFixed(1)} km`]]){const n=svgEl('text',{x,y,'font-size':13,fill:'#3d5b50'});n.textContent=t;svg.append(n);}return svg;
}
const measures=[['distance_m','Distance','km',1000],['distance_error_pct','Distance error','%',1],['ascent_100m_m','Ascent (100 m windows)','m',1],['steep_up_100m_m','Steep uphill distance','km',1000],['interior_reuse_geometry_m','Interior repetition','m',1],['shared_access_reuse_geometry_m','Shared access repetition','m',1],['surface_paved_m','Observed paved surface','km',1000],['surface_firm_unpaved_m','Observed firm unpaved','km',1000],['longest_paved_m','Longest paved section','km',1000],['high_traffic_estimate_m','Observed high traffic estimate','km',1000],['arterial_m','Observed arterial roads','km',1000],['tagged_signal_encounters','Tagged signal encounters','',1],['tagged_stop_encounters','Tagged stop encounters','',1],['tagged_barrier_encounters','Tagged barrier encounters','',1],['metadata_verified_pct','Verified segment metadata coverage','%',1],['surface_known_pct','Surface coverage','%',1],['traffic_estimate_known_pct','Traffic estimate coverage','%',1],['elevation_known_pct','Elevation coverage','%',1]];
function show(label,scroll=false){
  observers.forEach(o=>o.disconnect());observers=[];
  active=label;const c=data.cases.find(x=>x.label===label),box=$('detail');if(!c)return;
  document.querySelectorAll('.routeItem').forEach(b=>b.setAttribute('aria-pressed',b.dataset.case===label));
  box.replaceChildren(el('h2',pretty(label),'caseTitle'),el('span',`${c.group} · new capture ${c.status.toLowerCase()}`,'badge'));
  box.append(el('p',c.after?`New capture: ${fmt(c.after.requestMs,'s',1000)} generation; continuous cost ${fmt(c.after.cost)}, identical on a second walk. These are profile cost units, not a quality score.`:'New capture unavailable; historical geometry remains visible.'));
  if(c.defect){box.append(el('h3',c.defect.title,'mapHeading'),el('p',c.defect.text,'defectNote'),legend([['Frozen historical route',ORANGE],['Diagnosed location / segment',RED],...(c.defect.native.length?[['Captured native road shape',GREEN]]:[])]),map(c,true),el('p','Close-up shows the frozen evidence. The newly generated route is shown in the full overlay below.','muted'));}
  if(c.paired){const p=c.paired;box.append(el('p',`Latest same-request refinement rerun: ${p.outcome}. ${p.identical?'Original and published result were exactly identical.':'Geometry differs.'} ${p.reason}. This is separate from the fresh, refinement-off capture shown below.`,'notice'));}
  box.append(el('h3','Full route comparison','mapHeading'),legend([['Historical original',ORANGE],['New regeneration · refinement off',BLUE]]),map(c));
  const downloads=el('div',undefined,'downloads');downloads.append(link('Historical GPX',c.before.gpx));if(c.after)downloads.append(link('Regenerated GPX',c.after.gpx));box.append(downloads);
  const equal=c.after&&JSON.stringify(c.before.points)===JSON.stringify(c.after.points);
  box.append(el('p',equal?'The two saved point sequences are exactly identical. There is no geometry difference to spot.':'The two runs may follow different routes. Full-route differences include AUTO selection and timing effects; they do not isolate the construction fix.','muted'));
  box.append(el('h3','Elevation over distance'),elevation(c.after?[c.before,c.after]:[c.before]),el('p','Historical elevation is dashed orange; regenerated elevation is blue. Both share the same distance and elevation axes. Missing elevation breaks the line.','muted'));
  box.append(el('h3','Ride measurements and coverage'),el('p','Coverage differs between runs: the historical experiment collected unused OSM tags; the fresh capture matches the finished-route regression settings. Compare observed exposure only alongside coverage. Unknown does not mean zero, quiet or paved. Signals and barriers count available tags, not all real interruptions.','notice'));
  box.append(table(['Measurement','Historical','Regenerated'],measures.map(([key,name,unit,div])=>[name,fmt(c.before.metrics[key],unit,div),fmt(c.after?.metrics[key],unit,div)])));
  const details=el('details'),summary=el('summary','Case evidence and request settings');details.append(summary,el('p',`Profile ${c.profile}. AUTO, strict quality false, 60 s generation cap, refinement off. Historical target check ${c.testStatus}, ${c.testSeconds} s including pricing. Fresh capture time above measures generation only. No latency gain is inferred.`));if(c.defect)details.append(el('p',`Historical first pricing mismatch: point index ${c.defect.index}.`));box.append(details);
  if(scroll)$('explorer').scrollIntoView({behavior:'smooth'});
}
function filter(){const q=$('search').value.toLowerCase();const rows=data.cases.filter(c=>pretty(c.label).toLowerCase().includes(q)||c.label.includes(q));$('count').textContent=`${rows.length} of ${data.cases.length} cases · 14 historical finished-route failures, 6 smoke cases, 3 raw-seed cases`;
  $('routeList').replaceChildren(...rows.map(c=>{const b=el('button',undefined,'routeItem');b.dataset.case=c.label;b.setAttribute('aria-pressed',String(c.label===active));b.append(el('strong',pretty(c.label)),el('small',`${c.group} · ${c.status.toLowerCase()}`));b.onclick=()=>show(c.label);return b;}));}
$('search').oninput=filter;document.querySelectorAll('[data-case]').forEach(b=>b.onclick=()=>{$('search').value='';filter();show(b.dataset.case,true);});
$('captureNote').textContent=`Report built ${new Date(data.manifest.builtUtc).toLocaleString()}. ${data.manifest.geometryAudit.gpxFilesVerified} GPX files and ${data.manifest.geometryAudit.gpxPointsVerified.toLocaleString()} plotted coordinates audited. All ${data.manifest.geometryAudit.pricedCaptures} fresh captures priced twice. Source hashes and selection are retained in the manifest.`;
filter();show(active);
