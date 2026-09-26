'use strict';
const catalog = window.routeCatalog, $ = id => document.getElementById(id);
const NS = 'http://www.w3.org/2000/svg', colors = ['#bd530b','#007ba4','#713c9e','#27804c','#b83f6b','#6c6c16'];
let source = 'paired', activeId = '', requestId = 0;
function el(tag, text, cls) { const n = document.createElement(tag); if (text !== undefined) n.textContent = text; if (cls) n.className = cls; return n; }
function svgEl(tag, attrs) { const n = document.createElementNS(NS, tag); for (const [k,v] of Object.entries(attrs)) n.setAttribute(k,v); return n; }
function link(text, url) { const a = el('a',text); a.href = url; return a; }
function fmt(value, unit='', div=1) { return value === null || value === undefined || !Number.isFinite(Number(value)) ? 'unavailable' : `${(Number(value)/div).toLocaleString(undefined,{maximumFractionDigits:2})}${unit ? ' '+unit : ''}`; }
function table(headers, rows) { const wrap=el('div',undefined,'tableWrap'), t=el('table'), h=el('thead'), tr=el('tr'); headers.forEach(x=>tr.append(el('th',x))); h.append(tr); t.append(h); const body=el('tbody'); for(const row of rows){const r=el('tr'); row.forEach(x=>r.append(el('td',String(x))));body.append(r);}t.append(body);wrap.append(t);return wrap; }
function disclosure(title, contents) { const d=el('details'); d.append(el('summary',title),contents);return d; }
function pretty(label) { return label.replaceAll('_',' '); }
function setSource(next) {
  source=next; $('paired').setAttribute('aria-pressed',source==='paired');$('archive').setAttribute('aria-pressed',source==='archive');
  $('outcome').disabled=source==='archive';$('changedOnly').disabled=source==='archive';$('outcome').value='';$('profile').replaceChildren(new Option('All profiles',''));
  [...new Set(catalog.filter(x=>x.source===source).map(x=>x.profile))].sort().forEach(x=>$('profile').append(new Option(x,x)));
  $('scopeNote').textContent=source==='paired'?'All 460 paired requests and 920 original/result variants. Changed pairs appear first, alphabetically, so you can see a difference immediately. Unchanged and failed searches remain in the list.':'Historical recordings from the existing loop reports, including AUTO, GREEDY, ISO_GREEDY, probe and isochrone where saved. These runs have different provenance and are not a controlled comparison with the corrected experiment. Missing variants were not generated for this report.';
  activeId='';filter();
}
function filter() {
  const query=$('search').value.toLowerCase(),profile=$('profile').value,outcome=$('outcome').value;
  let rows=catalog.filter(x=>x.source===source && (!query||pretty(x.label).toLowerCase().includes(query)||x.label.includes(query)) && (!profile||x.profile===profile));
  if(source==='paired'&&outcome)rows=rows.filter(x=>outcome==='identical'?x.identical:x.outcome===outcome);
  rows.sort((a,b)=>(source==='paired'?Number(a.identical)-Number(b.identical):0)||a.label.localeCompare(b.label));
  $('count').textContent=source==='paired'?`${rows.length} of 460 pairs shown · ${rows.filter(x=>!x.identical).length} changed · ${rows.filter(x=>x.identical).length} identical`:`${rows.length} route requests · ${rows.reduce((s,x)=>s+x.variants.length,0)} saved recordings shown`;
  $('routeList').replaceChildren(...rows.map(row=>{const b=el('button',undefined,'routeItem');b.dataset.id=row.id;b.append(el('strong',pretty(row.label)));
    b.append(el('small',source==='paired'?(row.identical?`IDENTICAL · ${row.outcome}`:`CHANGED · ${fmt(row.addedM,'km',1000)} new geometry · ${fmt(row.gain,'%')} lower cost`):row.variants.join(' · ')));b.onclick=()=>load(row.id);return b;}));
  if(!rows.length){$('detail').replaceChildren(el('p','No matching routes. Reset filters to see all routes.'));activeId='';return;}
  load(rows.some(x=>x.id===activeId)?activeId:rows[0].id);
}
function load(id) {
  activeId=id; const token=++requestId; document.querySelectorAll('.routeItem').forEach(b=>b.setAttribute('aria-pressed',b.dataset.id===id));
  $('detail').replaceChildren(el('p','Loading route geometry…'));
  const script=document.createElement('script');script.src=`cases/${id}.js`;
  script.onload=()=>{const data=window.routeCase;script.remove();if(token!==requestId)return;if(!data||data.id!==id){$('detail').textContent='Route data mismatch.';return;}if(source==='paired')renderPair(data);else renderArchive(data);};
  script.onerror=()=>{script.remove();if(token===requestId)$('detail').textContent='Could not load route data. Keep the cases folder beside this report.';};document.body.append(script);
}
function projected(points,origin,latitude) { const sx=.111195*Math.cos(latitude*Math.PI/180);return points.map(p=>[(p[0]-origin[0])*sx,(p[1]-origin[1])*.111195]); }
function pathData(lines) { return lines.map(line=>line.map((p,i)=>`${i?'L':'M'}${p[0].toFixed(2)},${(-p[1]).toFixed(2)}`).join(' ')).join(' '); }
function bounds(lines) { let x0=Infinity,y0=Infinity,x1=-Infinity,y1=-Infinity;for(const line of lines)for(const[x,y]of line){x0=Math.min(x0,x);x1=Math.max(x1,x);y0=Math.min(y0,-y);y1=Math.max(y1,-y);}if(!Number.isFinite(x0))return[-1,-1,2,2];let w=Math.max(100,x1-x0)*1.12,h=Math.max(100,y1-y0)*1.12;if(w/h<5/3)w=h*5/3;else h=w*3/5;return[(x0+x1-w)/2,(y0+y1-h)/2,w,h]; }
function mapWidget(layers,changed) {
  const wrap=el('div'), controls=el('div',undefined,'mapControls'),svg=svgEl('svg',{class:'map',height:560,role:'img','aria-label':changed?'Route overlay. Orange original-only geometry, blue result-only geometry, grey shared geometry.':'Saved algorithm variants on shared map coordinates'});
  const full=bounds(layers.flatMap(l=>l.lines)), changedBounds=changed&&changed.length?bounds(changed):full;
  let view=full.slice(),drag=null;
  function update(){svg.setAttribute('viewBox',view.join(' '));}
  for(const layer of layers){svg.append(svgEl('path',{d:pathData(layer.lines),fill:'none',stroke:layer.color,'stroke-width':layer.width||3,'stroke-linecap':'round','stroke-linejoin':'round','vector-effect':'non-scaling-stroke',opacity:layer.opacity||1}));}
  const first=layers.find(l=>l.lines[0]?.length)?.lines[0][0];if(first)svg.append(svgEl('path',{d:`M${first[0]},${-first[1]} l0,0`,stroke:'#182b33','stroke-width':11,'stroke-linecap':'round','vector-effect':'non-scaling-stroke'}));
  const fit=el('button','Whole route'),change=el('button','Fit changed sections');fit.onclick=()=>{view=full.slice();update();};change.disabled=!changed?.length;change.onclick=()=>{view=changedBounds.slice();update();};
  function pointer(e){const p=svg.createSVGPoint();p.x=e.clientX;p.y=e.clientY;return p.matrixTransform(svg.getScreenCTM().inverse());}
  svg.addEventListener('wheel',e=>{e.preventDefault();const p=pointer(e),factor=e.deltaY>0?1.2:1/1.2;if(view[2]*factor<30||view[2]*factor>full[2]*10)return;view=[p.x+(view[0]-p.x)*factor,p.y+(view[1]-p.y)*factor,view[2]*factor,view[3]*factor];update();},{passive:false});
  svg.onpointerdown=e=>{drag={point:pointer(e),view:view.slice()};svg.setPointerCapture(e.pointerId);};svg.onpointermove=e=>{if(!drag)return;const p=pointer(e);view[0]+=drag.point.x-p.x;view[1]+=drag.point.y-p.y;update();};svg.onpointerup=svg.onpointercancel=()=>{drag=null;};
  controls.append(fit,change,el('span','North ↑ · black dot = start · drag to pan, scroll to zoom','muted'));wrap.append(controls,svg);update();return wrap;
}
function legend(items){const l=el('div',undefined,'legend');for(const[text,color]of items){const s=el('span',text);s.style.setProperty('--color',color);l.append(s);}return l;}
function distance(a,b){const lat=(a[1]+b[1])/2e6-90;return Math.hypot((a[0]-b[0])*.111195*Math.cos(lat*Math.PI/180),(a[1]-b[1])*.111195);}
function elevation(routes){const profiles=routes.map(r=>{let d=0;return r.points.map((p,i)=>{if(i)d+=distance(r.points[i-1],p);return[d,p[2]];});});let maxD=1,low=Infinity,high=-Infinity;for(const p of profiles)for(const[d,e]of p){maxD=Math.max(maxD,d);if(e!==null&&Number.isFinite(e)){low=Math.min(low,e);high=Math.max(high,e);}}if(!Number.isFinite(low))return el('p','Elevation unavailable.');const svg=svgEl('svg',{viewBox:'0 0 1000 220',class:'elevation',role:'img','aria-label':'Route elevation profiles on shared axes'});low-=10;high=Math.max(high+10,low+30);
  profiles.forEach((points,i)=>{let active=false,d='';for(const[dist,e]of points){if(e===null||!Number.isFinite(e)){active=false;continue;}d+=`${active?'L':'M'}${45+dist/maxD*930},${185-(e-low)/(high-low)*165}`;active=true;}svg.append(svgEl('path',{d,fill:'none',stroke:routes[i].color,'stroke-width':2,'stroke-dasharray':i===0?'6 3':'none'}));});
  for(const[x,y,text]of [[4,18,`${Math.round(high)} m`],[4,185,`${Math.round(low)} m`],[45,212,'0 km'],[875,212,`${(maxD/1000).toFixed(1)} km`]]){const t=svgEl('text',{x,y,'font-size':13,fill:'#344954'});t.textContent=text;svg.append(t);}return svg;
}
const metrics=[['distance_m','Distance','km',1000],['distance_error_pct','Distance error','%',1],['surface_paved_m','Observed paved surface','km',1000],['surface_firm_unpaved_m','Observed firm unpaved','km',1000],['poor_smoothness_m','Observed rough surface','km',1000],['high_traffic_estimate_m','Observed high traffic estimate','km',1000],['arterial_m','Observed arterial roads','km',1000],['longest_paved_m','Longest continuous paved section','km',1000],['longest_firm_unpaved_m','Longest continuous firm unpaved section','km',1000],['ascent_100m_m','Ascent in 100 m windows','m',1],['steep_up_100m_m','Steep uphill distance','km',1000],['tagged_signal_encounters','Mapped signal encounters','',1],['tagged_stop_encounters','Mapped stop encounters','',1],['tagged_barrier_encounters','Mapped barrier encounters','',1],['sharp_geometry_bends','Sharp geometry bends','',1],['geometry_uturns','Geometry U-turns','',1],['interior_reuse_geometry_m','Interior repetition','m',1],['shared_access_reuse_geometry_m','Shared access repetition','m',1],['surface_known_pct','Surface coverage','%',1],['traffic_estimate_known_pct','Traffic estimate coverage','%',1],['smoothness_known_pct','Smoothness coverage','%',1],['elevation_known_pct','Elevation coverage','%',1]];
function metricTable(definitions,before,after){return table(['Measurement','Original','Result','Result − original'],definitions.map(([key,label,unit,div])=>{const a=before[key],b=after[key];return[label,fmt(a,unit,div),fmt(b,unit,div),a==null||b==null?'unavailable':fmt(b-a,unit==='%'?'points':unit,div)];}));}
function renderPair(d){const box=$('detail');box.replaceChildren(el('h2',pretty(d.label)));const status=el('span',d.identical?'IDENTICAL GEOMETRY':'CHANGED GEOMETRY',`tag ${d.identical?'':'changed'}`);box.append(status,el('span',d.outcome,'tag'));
  box.append(el('p',d.identical?'There is no route difference to spot in this pair. Refinement retained the original route. Both GPX files describe the same geometry.':`${fmt(d.addedM,'km',1000)} of result geometry is outside the original corridor; ${fmt(d.removedM,'km',1000)} of original geometry is outside the result corridor. Profile cost per metre is ${fmt(d.gain,'%')} lower. This does not establish a cyclist preference.`,`notice ${d.identical?'identical':''}`));
  box.append(el('p',`Outcome: ${d.reason}. Full request ${fmt(d.requestMs,'s',1000)}; refinement ${fmt(d.addedMs,'s',1000)}.`));
  const before=projected(d.before.points,d.origin,d.latitude),after=projected(d.after.points,d.origin,d.latitude);
  box.append(legend([['Shared geometry / retained route','#9aa9b0'],['Only on original, within approximation','#bd530b'],['Only on result, within approximation','#007ba4']]));
  box.append(mapWidget([{lines:[before],color:'#9aa9b0',width:3},{lines:[after],color:'#9aa9b0',width:3},{lines:d.removed,color:colors[0],width:4},{lines:d.added,color:colors[1],width:4}],[...d.removed,...d.added]));
  const downloads=el('div',undefined,'downloads');downloads.append(link('Original GPX',d.before.gpx),link('Result GPX',d.after.gpx));box.append(downloads,el('h3','Elevation over distance'),legend([['Original, dashed',colors[0]],['Result',colors[1]]]),elevation([{points:d.before.points,color:colors[0]},{points:d.after.points,color:colors[1]}]));
  box.append(el('p','The elevation curves use distance from the start. Route changes can shift corresponding landmarks horizontally. Missing elevation samples break the line.','muted'));
  box.append(el('h3','What would the rider notice?'),el('p','Read the changes alongside coverage. More climbing, more gravel or fewer bends may suit one rider and not another. Traffic is an estimate; unknown roads are not assumed quiet. Tagged interruptions are incomplete.','muted'),metricTable(metrics,d.before.metrics,d.after.metrics));
  box.append(disclosure('All recorded ride measurements',metricTable(Object.keys(d.before.metrics).map(k=>[k,k,'',1]),d.before.metrics,d.after.metrics)));
  const details=el('div');details.append(table(['Property','Value'],Object.entries(d.cell)),el('p','Cost values are profile costs, not enjoyment scores. Historical track costs are not the continuous oracle used for acceptance.'));
  box.append(disclosure('Request, producing tier, pricing and timings',details));
  const search=el('div');search.append(el('p',`${d.cell.proposals} proposals; ${d.cell.evaluations} evaluations attempted; ${d.traces.length} recorded evaluation rows; ${d.cell.finalizations} finalizations. A search-state acceptance is not publication of a finished route. Rejected candidate geometry was not retained.`));search.append(table(['Rejection reason','Count'],d.rejections.map(x=>[x.reason,x.count])),table(['Evaluation','Operator','Search cost/m','Search state accepted','Marked feasible'],d.traces.map(x=>[x.evaluation,x.operator,fmt(x.energy),x.accepted,x.feasible])));box.append(disclosure('All saved candidate traces and rejection counts',search));
}
function renderArchive(d){const box=$('detail');box.replaceChildren(el('h2',pretty(d.label)),el('p','Historical recordings. Dates and inputs differ from the corrected experiment. These variants are available for visual inspection only; do not infer refinement gains by comparing them with current pairs.','notice'));const options=el('div',undefined,'archivedOptions'),drawing=el('div');const origin=[d.recordings[0].geometry.coordinates[0][0]*1e6+180e6,d.recordings[0].geometry.coordinates[0][1]*1e6+90e6],latitude=origin[1]/1e6-90;const selected=new Set(d.recordings.map((_,i)=>i));
  function draw(){const items=[...selected].map(i=>{const r=d.recordings[i],points=r.geometry.coordinates.map(p=>[Math.round((p[0]+180)*1e6),Math.round((p[1]+90)*1e6),p[2]??null]);return{points,color:colors[i%colors.length],name:r.variant,lines:[projected(points,origin,latitude)]};});drawing.replaceChildren();if(items.length)drawing.append(legend(items.map(x=>[x.name,x.color])),mapWidget(items),elevation(items));}
  d.recordings.forEach((r,i)=>{const label=el('label'),check=el('input');check.type='checkbox';check.checked=true;check.onchange=()=>{if(check.checked)selected.add(i);else selected.delete(i);draw();};label.append(check,document.createTextNode(`${r.variant} · file modified ${new Date(r.sourceMtime*1000).toLocaleString()} · `),link('Source GeoJSON',r.sourceFile));options.append(label);});box.append(options,drawing);draw();
  box.append(disclosure('Recorded properties',table(['Recording','Property','Value'],d.recordings.flatMap((r,i)=>Object.entries(r.properties).filter(([k])=>k!=='messages').map(([k,v])=>[`${i+1}: ${r.variant}`,k,typeof v==='object'?JSON.stringify(v):v])))));
}
$('paired').onclick=()=>setSource('paired');$('archive').onclick=()=>setSource('archive');$('search').oninput=filter;$('profile').onchange=filter;$('outcome').onchange=filter;
$('changedOnly').onclick=()=>{$('search').value='';$('profile').value='';$('outcome').value='accepted';filter();};$('reset').onclick=()=>{$('search').value='';$('profile').value='';$('outcome').value='';filter();};
setSource('paired');
