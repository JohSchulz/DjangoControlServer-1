const IndexTable = Vue.createApp({});

IndexTable.component('index-table', {
	props: ['devices'],
	template: `
	<table class="table table-striped">
		<thead class="thead-dark">
			<tr>
			<th>#</th>
			<th>Name</th>
			<th>IP</th>
			<th>Status</th>
			<th colspan=2></th>
			</tr>
		</thead>
		<tbody>
		<device-widget v-for="d in devices" v-bind:device="d" :key="d.pk"></device-widget>
		</tbody>
	</table>
	`,
	mounted () {
		// this.get_devices(); currently, the index is load with django template tags
	},
	methods: {
		get_devices() {
			config = {	method : 'GET',
						url : '/devices/',
						xsrfCookieName: 'csrftoken',
						xsrfHeaderName: 'X-CSRFTOKEN', };
			axios(config)
				.then(response => {
					console.log(response);
					this.devices = response.data})
				.then(error => console.log(error));
		},
	},
})

IndexTable.component('device-widget', {
	data ()  { return {
		status : [],
		config : [],
		}
	},
	props: ['device'],
	template: `
	<tr>
	<td>{{ this.device.model }} # {{ this.device.pk }}</td>
	<td><a v-bind:href="'/' + this.device.model + '/' + this.device.pk + '/'">{{ this.device.fields.name }}</a></td>
	<td>{{ this.device.fields.ip }}</td>
	<td>{{ this.status }}</td>
	<!-- td><button class="btn btn-outline-primary" v-on:click="init_device(device.model, device.pk)">Status</button></td -->
	<td><<button class="btn btn-primary" v-on:click=this.detail_device()>Details</button></td>
	<td><button class="btn btn-warning">Remove</button></td>
	</tr>
	`,
	mounted () {
		this.init_device(this.device.model, this.device.pk);
	},
	methods: {
		init_device(model, pk) {
			config = {	method : 'POST',
					url : '/device/',
					xsrfCookieName: 'csrftoken',
					xsrfHeaderName: 'X-CSRFTOKEN',
					data : [model, pk, 'STATUS'] };
			this.config = config;
			var sstatus = {};
			axios(config)
				.then(response => { 
					this.status = response.data['message'];
				})
				.catch(error => console.log(error));
		},
		remove_device() {
			config = this.config;
			config['data'][2] = 'DELETE';
			axios(config)
				.then(response => console.log(response))
				.catch(error => console.log(error));
		},
		detail_device() {
			window.location.replace(this.device.model + '/' + this.device.pk + '/');
			/*
			config = this.config;
			config['data'][2] = 'DETAIL';
			axios(config)
				.then(response => {
					console.log(response); 
					//this.detail = response.data; 
					})
				.catch(error => console.log(error)); */
		},
	},
})

IndexTable.mount('#index')
